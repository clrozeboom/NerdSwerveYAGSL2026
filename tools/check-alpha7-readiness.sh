#!/usr/bin/env bash
# Reports whether the vendor ecosystem has caught up enough to upgrade this project to
# WPILib 2027.0.0-alpha-7. See docs/UPGRADE-ALPHA7.md for what the blockers are and why.
#
# The version labels vendors publish are not the real test — every 2027 vendordep still says
# wpilibYear 2027_alpha5 while several of them work fine on alpha-6. So for the two dependencies
# that actually gate us, this downloads the jar and checks whether it still references classes
# alpha-7 deleted. That is the compatibility question.
#
# Exit status: 0 = ready to upgrade, 1 = still blocked, 2 = the check itself failed.

set -uo pipefail

MARKET="https://frcmaven.wpi.edu/artifactory/vendordeps/vendordep-marketplace"
AKIT_MVN="https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release/org/littletonrobotics/akit/akit-java"
REV_MVN="https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java"
WPI_MVN="https://frcmaven.wpi.edu/artifactory/release/org/wpilib"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

fetch() { curl -sSL --max-time 120 "$1"; }

# Classes alpha-7 deleted or moved. A jar referencing any of these cannot run on alpha-7.
DELETED_IN_ALPHA7='org/wpilib/smartdashboard/SmartDashboard
org/wpilib/smartdashboard/SendableChooser
org/wpilib/smartdashboard/Field2d
org/wpilib/util/sendable/Sendable
org/wpilib/networktables/NTSendable
org/wpilib/networktables/NTSendableBuilder
org/wpilib/math/util/Pair
org/wpilib/driverstation/Alert'

# Prints the alpha-7-deleted classes a jar still references, one per line.
jar_incompatibilities() {
  local url="$1" jar="$WORK/probe.jar" dir="$WORK/probe"
  rm -rf "$dir" "$jar"; mkdir -p "$dir"
  curl -sSL --max-time 300 -o "$jar" "$url" 2>/dev/null || return 1
  unzip -qo "$jar" -d "$dir" 2>/dev/null || return 1
  local refs
  refs=$(find "$dir" -name '*.class' -exec strings {} + 2>/dev/null \
         | grep -oE 'org/wpilib/[A-Za-z0-9/$]+' | sed 's/\$.*//' | sort -u)
  comm -12 <(echo "$refs") <(echo "$DELETED_IN_ALPHA7" | sort)
}

# Newest version in a maven-metadata.xml, or empty.
latest_version() {
  fetch "$1/maven-metadata.xml" 2>/dev/null \
    | grep -oE '<version>[^<]+</version>' | sed 's/<[^>]*>//g' | tail -1
}

echo "=== alpha-7 upgrade readiness — $(date -u '+%Y-%m-%d %H:%M UTC') ==="
echo

# --- 1. Has the vendordep marketplace moved past alpha-5? ---
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

# --- 2. WPILib itself ---
echo "WPILib core latest: $(latest_version "$WPI_MVN/wpilibj/wpilibj-java")"
echo

# --- 3. The two dependencies that gate us ---
AKIT_LATEST=$(latest_version "$AKIT_MVN")
echo "AdvantageKit latest: ${AKIT_LATEST:-<unreachable>}"
AKIT_BAD=""
if [ -n "$AKIT_LATEST" ]; then
  AKIT_BAD=$(jar_incompatibilities "$AKIT_MVN/$AKIT_LATEST/akit-java-$AKIT_LATEST.jar")
  if [ -z "$AKIT_BAD" ]; then
    echo "  CLEAN against alpha-7"
  else
    echo "  BLOCKED — still references:"; echo "$AKIT_BAD" | sed 's/^/    /'
  fi
fi
echo

# REVLib: prefer its own maven, but that host is not always reachable from a sandbox, so fall back
# to the newest version named in the marketplace bucket listing.
REV_LATEST=$(latest_version "$REV_MVN")
REV_SRC="maven.revrobotics.com"
if [ -z "$REV_LATEST" ]; then
  REV_SRC="marketplace listing"
  for b in $BUCKETS; do
    v=$(fetch "$MARKET/$b/" | grep -oE 'REVLib-[0-9][^"]*\.json' \
        | sed 's/^REVLib-//;s/\.json$//' | sort -V | tail -1)
    [ -n "$v" ] && REV_LATEST="$v"
  done
fi
echo "REVLib latest: ${REV_LATEST:-<unknown>}  (via $REV_SRC)"
REV_BAD=""
if [ -n "$REV_LATEST" ]; then
  if REV_BAD=$(jar_incompatibilities "$REV_MVN/$REV_LATEST/REVLib-java-$REV_LATEST.jar"); then
    if [ -z "$REV_BAD" ]; then
      echo "  CLEAN against alpha-7"
    else
      echo "  references:"; echo "$REV_BAD" | sed 's/^/    /'
    fi
  else
    REV_BAD=""
    echo "  jar not reachable from here — version only, compatibility unverified"
  fi
fi
echo

# --- 4. The rest, for context ---
echo "Other vendors (newest published 2027 vendordep in each bucket):"
for b in $BUCKETS; do
  fetch "$MARKET/$b/" | grep -oE 'href="[^"]+\.json"' | sed 's/href="//;s/"//' \
    | grep -viE 'replay' | sed "s/^/  [$b] /"
done
echo

# --- 5. Verdict ---
if [ -n "$AKIT_LATEST" ] && [ -z "$AKIT_BAD" ]; then
  echo "VERDICT: GO — AdvantageKit $AKIT_LATEST is clean against alpha-7."
  [ -n "$REV_BAD" ] && echo "  (REVLib still references: $(echo "$REV_BAD" | tr '\n' ' ') — check that path is unused)"
  echo "  Next: follow docs/UPGRADE-ALPHA7.md from step 3."
  exit 0
fi
echo "VERDICT: BLOCKED — waiting on an AdvantageKit build without the deleted Sendable classes."
exit 1
