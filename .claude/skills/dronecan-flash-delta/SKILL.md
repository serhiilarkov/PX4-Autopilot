---
name: dronecan-flash-delta
description: Build a PX4 board and report the DroneCAN/UAVCAN subsystem flash footprint (text+rodata), optionally as a delta between two git refs — the acceptance metric for the libcanard migration
argument-hint: "<board=ark_fmu-v6x_default> [ref-before] [ref-after]"
allowed-tools: Bash, Read
---

# DroneCAN flash delta

Measure the flash (`.text`+`.rodata`) the DroneCAN/UAVCAN subsystem consumes on a PX4 board, using the reproducible bucketing from `DRONECAN_LIBCANARD_MIGRATION.md` §9.1. This is the **acceptance metric** for the libuavcan→libcanard migration (target: ~64 KB reclaimed on `ark_fmu-v6x`: ~124 KB → ~53 KB).

## Arguments
- `$1` — board, default `ark_fmu-v6x_default`.
- `$2`, `$3` — optional git refs. If both are given, build each and report the delta (before → after). If omitted, measure the current tree.

## Procedure

1. **Resolve board:** `BOARD=${1:-ark_fmu-v6x_default}`.

2. **Build.** Single measurement: `make $BOARD` → ELF at `build/$BOARD/$BOARD.elf`. For a delta, build each ref in isolation so the user's tree is untouched:
   ```bash
   git worktree add /tmp/fd-before "$2" && make -C /tmp/fd-before $BOARD
   git worktree add /tmp/fd-after  "$3" && make -C /tmp/fd-after  $BOARD
   # measure both ELFs, then: git worktree remove /tmp/fd-before /tmp/fd-after
   ```

3. **Bucket the symbols** (sized text/rodata, dedup by address, bucket by demangled name). Run against each ELF — set `ELF` first:
   ```bash
   arm-none-eabi-nm --print-size --radix=d --demangle "$ELF" \
     | awk 'NF==4 && $3 ~ /^[TtWwRr]$/ && !seen[$1]++ {
         n=$4
         if      (n ~ /^uavcan::/)                              b="libuavcan (lib)"
         else if (n ~ /^uavcan_stm32h7::/)                      b="old CAN driver"
         else if (n ~ /Uavcan|uavcan_main/)                     b="old bridge (uavcan)"
         else if (n ~ /^canard/)                                b="libcanard core"
         else if (n ~ /[Dd]ronecan/)                            b="new driver (dronecan)"
         else if (n ~ /^(ardupilot|com|cuav|mppt|dronecan|uavcan)_/) b="generated codecs"
         else next
         sz[b]+=$2; tot+=$2
       } END { for (k in sz) printf "  %-22s %9d B\n", k, sz[k]
               printf "  %-22s %9d B\n", "TOTAL DroneCAN", tot }'
   ```

4. **Report.** Print the per-bucket table for each ELF. For a delta, show before, after, and `after − before` for the DroneCAN total. Also report the board's overall flash headroom (`size build/$BOARD/$BOARD.elf`, or the `make` tail).

## Notes
- Needs the PX4 ARM toolchain (`arm-none-eabi-nm`, `arm-none-eabi-size`) on PATH.
- Both the **old** (`uavcan::`, `Uavcan*`) and **new** (`canard*`, `dronecan`) buckets are matched, so the skill works before, during, and after the migration — a transitional board carries only one of them.
- `ark_fmu-v6x` is the canonical constrained target: it fits, so the ELF is real and the before/after numbers are trustworthy. Prior baseline + ICF/de-templatization findings: memory `project-flash-bloat-icf`.
