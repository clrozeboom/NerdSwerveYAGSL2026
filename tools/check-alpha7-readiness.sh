#!/usr/bin/env bash
# Reports whether the vendor ecosystem has caught up enough to move a 2027 robot project from
# WPILib 2027.0.0-alpha-6 to alpha-7. See docs/UPGRADE-ALPHA7.md for what the blockers are and why.
#
# Two projects share this check, because they share the WPILib gate and differ only in which
# vendordeps they pin:
#
#   NerdSwerveYAGSL2026   AdvantageKit, REVLib
#   BasicRobotLessons     Phoenix6, PhotonLib
#
# The version labels vendors publish are not the test. Every 2027 vendordep still declares
# wpilibYear 2027_alpha5, yet several of them run fine on alpha-6 — because alpha-6 happened to keep
# every class those builds reference. alpha-7 deletes the whole Sendable world, so the real question
# is per-jar: does this artifact still reference a class alpha-7 no longer ships?
#
# So that is what this measures. It builds a class index from the alpha-7 artifacts themselves
# rather than carrying a hand-maintained list of deletions, which means it stays honest when
# alpha-8 lands and deletes something nobody has written down yet.
#
# Exit status: 0 = every vendor clean, 1 = at least one still blocked, 2 = the check itself failed.

set -uo pipefail

MARKET="https://frcmaven.wpi.edu/artifactory/vendordeps/vendordep-marketplace"
WPI_MVN="https://frcmaven.wpi.edu/artifactory/release/org/wpilib"

# The org/wpilib java artifacts a robot project can see on the classpath. The index built from these
# defines "exists in alpha-7"; a reference to anything outside it is a blocker.
WPI_ARTIFACTS='wpilibj/wpilibj-java
wpimath/wpimath-java
wpiutil/wpiutil-java
wpiunits/wpiunits-java
wpinet/wpinet-java
ntcore/ntcore-java
hal/hal-java
cscore/cscore-java
cameraserver/cameraserver-java
apriltag/apriltag-java
fields/fields-java
datalog/datalog-java
drivers/drivers-java
telemetry/telemetry-java
tunables/tunables-java
epilogue/epilogue-runtime-java
annotations-java
commandsv2/commandsv2-java
commandsv3-java
romiVendordep/romiVendordep-java
xrpVendordep/xrpVendordep-java'

# label | maven base | artifact id | project
VENDORS='AdvantageKit|https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release/org/littletonrobotics/akit/akit-java|akit-java|NerdSwerveYAGSL2026
REVLib|https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java|REVLib-java|NerdSwerveYAGSL2026
Phoenix6|https://maven.ctr-electronics.com/release/com/ctre/phoenix6/wpiapi-java|wpiapi-java|BasicRobotLessons
PhotonLib|https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-java|photonlib-java|BasicRobotLessons'

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

fetch() { curl -sSL --max-time 120 "$1"; }

# Newest version in a maven-metadata.xml, or empty if the host is unreachable.
latest_version() {
  fetch "$1/maven-metadata.xml" 2>/dev/null \
    | grep -oE '<version>[^<]+</version>' | sed 's/<[^>]*>//g' | tail -1
}

# Every org/wpilib class name a jar's constant pool mentions, one per line, outer classes only.
# Takes a URL or a local path.
jar_refs() {
  local src="$1" jar="$WORK/probe.jar" dir="$WORK/probe"
  rm -rf "$dir" "$jar"; mkdir -p "$dir"
  if [ -f "$src" ]; then
    cp "$src" "$jar"
  else
    curl -sSL --max-time 300 -o "$jar" "$src" 2>/dev/null || return 1
  fi
  unzip -qo "$jar" -d "$dir" 2>/dev/null || return 1
  find "$dir" -name '*.class' -exec strings {} + 2>/dev/null \
    | grep -oE 'org/wpilib/[A-Za-z0-9/$]+' | sed 's/\$.*//' | sort -u
}

# Newest copy of an artifact already in the Gradle cache, or empty.
#
# REVLib publishes only to maven.revrobotics.com, which some sandboxes cannot reach, and its
# marketplace JSON is named with a different version string than the maven artifact
# ("2027.0.0-alpha6" vs "2027.0.0-alpha-6"), so the URL cannot be reconstructed from the listing
# either. A jar the build already pulled down is the same bytes and answers the same question.
cached_jar() {
  find "${GRADLE_USER_HOME:-$HOME/.gradle}" /root/.gradle -type f -name "$1-*.jar" 2>/dev/null \
    | grep -v -- '-sources\|-javadoc' | sort -V | tail -1
}

echo "=== alpha-7 upgrade readiness — $(date -u '+%Y-%m-%d %H:%M UTC') ==="
echo

# --- 1. WPILib itself, and the class index everything is measured against ---
WPI_LATEST=$(latest_version "$WPI_MVN/wpilibj/wpilibj-java")
echo "WPILib core latest: ${WPI_LATEST:-<unreachable>}"
if [ -z "$WPI_LATEST" ]; then
  echo "ERROR: cannot reach frcmaven — nothing to measure against."
  exit 2
fi

INDEX="$WORK/wpilib-classes.txt"
: > "$INDEX"
INDEXED=0
for a in $WPI_ARTIFACTS; do
  name="${a##*/}"
  jar="$WORK/idx.jar"; dir="$WORK/idx"
  rm -rf "$dir" "$jar"; mkdir -p "$dir"
  curl -sSL --max-time 300 -o "$jar" "$WPI_MVN/$a/$WPI_LATEST/$name-$WPI_LATEST.jar" 2>/dev/null || continue
  unzip -qo "$jar" -d "$dir" 2>/dev/null || continue
  (cd "$dir" && find . -name '*.class' | sed 's|^\./||;s|\.class$||;s|\$.*||') >> "$INDEX"
  INDEXED=$((INDEXED + 1))
done
sort -u -o "$INDEX" "$INDEX"
CLASS_COUNT=$(wc -l < "$INDEX")
echo "  indexed $INDEXED/$(echo "$WPI_ARTIFACTS" | wc -l) artifacts, $CLASS_COUNT classes"

# A partial index would call every vendor blocked for the wrong reason, so refuse rather than lie.
# The anchors are three classes from three different artifacts that any usable index must contain.
TOTAL_ARTIFACTS=$(echo "$WPI_ARTIFACTS" | wc -l)
ANCHORS_OK=1
for c in org/wpilib/framework/TimedRobot org/wpilib/math/geometry/Rotation2d org/wpilib/telemetry/Telemetry; do
  grep -qxF "$c" "$INDEX" || { ANCHORS_OK=0; echo "  missing anchor: $c"; }
done
if [ "$INDEXED" -lt "$TOTAL_ARTIFACTS" ] || [ "$ANCHORS_OK" -eq 0 ]; then
  echo "ERROR: alpha-7 class index is incomplete — not reporting verdicts off it."
  exit 2
fi
echo

# --- 2. Has the vendordep marketplace moved past alpha-5? ---
BUCKETS=$(fetch "$MARKET/" | grep -oE 'href="2027[^"]*/"' | sed 's/href="//;s/\///;s/"//' | sort -u)
echo "Vendordep marketplace buckets: $(echo "$BUCKETS" | tr '\n' ' ')"
NEW_BUCKET=""
for b in $BUCKETS; do
  case "$b" in *alpha6*|*alpha7*) NEW_BUCKET="$NEW_BUCKET $b" ;; esac
done
if [ -n "$NEW_BUCKET" ]; then
  echo "  ** NEW: a post-alpha-5 bucket exists:$NEW_BUCKET"
else
  echo "  still alpha-5 only — vendors have not rebuilt for alpha-6/7 yet"
fi
echo

# --- 3. Each pinned vendordep, tested against the index ---
BLOCKED_NERD=""; BLOCKED_LESSONS=""
UNKNOWN_NERD=""; UNKNOWN_LESSONS=""
SUMMARY=""

echo "Pinned vendordeps:"
while IFS='|' read -r label base artifact project; do
  [ -n "$label" ] || continue
  ver=$(latest_version "$base")

  # When the vendor's own maven is unreachable, the marketplace listing at least names a version.
  if [ -z "$ver" ]; then
    for b in $BUCKETS; do
      v=$(fetch "$MARKET/$b/" | grep -oE "${label}-[0-9][^\"]*\.json" \
          | sed "s/^${label}-//;s/\.json$//" | sort -V | tail -1)
      [ -n "$v" ] && ver="$v"
    done
  fi

  if [ -z "$ver" ]; then
    printf '  %-13s %-31s %s\n' "$label" "<unreachable>" "[$project]"
    case "$project" in
      NerdSwerve*) UNKNOWN_NERD="$UNKNOWN_NERD $label" ;;
      *)           UNKNOWN_LESSONS="$UNKNOWN_LESSONS $label" ;;
    esac
    continue
  fi

  measured="$ver"
  refs=$(jar_refs "$base/$ver/$artifact-$ver.jar")
  if [ -z "$refs" ]; then
    local_jar=$(cached_jar "$artifact")
    if [ -n "$local_jar" ]; then
      refs=$(jar_refs "$local_jar")
      measured="$(basename "$local_jar" .jar | sed "s/^$artifact-//") (gradle cache)"
    fi
  fi
  if [ -z "$refs" ]; then
    printf '  %-13s %-31s [%s] jar unreachable — version only, compatibility unverified\n' \
      "$label" "$ver" "$project"
    case "$project" in
      NerdSwerve*) UNKNOWN_NERD="$UNKNOWN_NERD $label" ;;
      *)           UNKNOWN_LESSONS="$UNKNOWN_LESSONS $label" ;;
    esac
    continue
  fi

  missing=$(comm -23 <(echo "$refs") "$INDEX")
  if [ -z "$missing" ]; then
    printf '  %-13s %-31s [%s] CLEAN against alpha-7\n' "$label" "$measured" "$project"
    SUMMARY="$SUMMARY$label $measured clean; "
  else
    n=$(echo "$missing" | wc -l)
    printf '  %-13s %-31s [%s] BLOCKED — %s class(es) missing in alpha-7:\n' \
      "$label" "$measured" "$project" "$n"
    echo "$missing" | sed 's|^org/wpilib/|      |'
    SUMMARY="$SUMMARY$label $measured blocked ($n); "
    case "$project" in
      NerdSwerve*) BLOCKED_NERD="$BLOCKED_NERD $label" ;;
      *)           BLOCKED_LESSONS="$BLOCKED_LESSONS $label" ;;
    esac
  fi
done <<< "$VENDORS"
echo

# --- 4. The rest, for context ---
echo "Other vendors (newest published 2027 vendordep in each bucket):"
for b in $BUCKETS; do
  fetch "$MARKET/$b/" | grep -oE 'href="[^"]+\.json"' | sed 's/href="//;s/"//' \
    | grep -viE 'replay' | sed "s/^/  [$b] /"
done
echo

# --- 5. Per-project verdicts ---
verdict() {
  local project="$1" blocked="$2" unknown="$3" doc="$4"
  if [ -n "$blocked" ]; then
    echo "$project: BLOCKED — waiting on$blocked"
  elif [ -n "$unknown" ]; then
    echo "$project: UNVERIFIED — could not fetch$unknown; re-run when the host is reachable"
  else
    echo "$project: GO — every pinned vendordep is clean against alpha-7. Next: $doc"
  fi
}

echo "VERDICT"
verdict "NerdSwerveYAGSL2026" "$BLOCKED_NERD" "$UNKNOWN_NERD" "docs/UPGRADE-ALPHA7.md from step 3"
verdict "BasicRobotLessons " "$BLOCKED_LESSONS" "$UNKNOWN_LESSONS" "docs/lesson-plan-alpha7-upgrade.md"
echo
echo "SUMMARY: WPILib $WPI_LATEST; $SUMMARY"

[ -n "$BLOCKED_NERD$BLOCKED_LESSONS" ] && exit 1
exit 0
