# Gold sets — translation regression checks

A gold set pins source strings where the model or the prompt has already failed,
plus explicit terminology decisions. A translation that differs from `expected_ru`
is a regression: it appears as an M1 hit in the review report, not a silent change.

## Files

| File | Purpose |
|---|---|
| `ru.gold.tsv` | Russian gold set, four TAB-separated columns: `id`, `source`, `expected_ru`, `category` |

## Format

- `id` — stable (`GS-NNN`). The set grows by adding rows only; ids are never reused.
- `source` / `expected_ru` — the strings exactly as stored in the `.ts` catalog:
  HTML entities as written in the file (`couldn&apos;t`), embedded tabs escaped as
  two-character `\t` so the TSV columns stay intact (this matches the escaping
  `dump.py` prints, one for one).
- `category` — the term or rule class the row guards (elevation, release, pin, …).

## Checking a catalog

Dump the catalog with `dump.py`, then compare each gold `source` against its
`expected_ru`; any mismatch is reported as a regression. The set is a check on the
pipeline (model + prompt), not a queue of edits: fixing a hit means fixing the
prompt or the table, re-running, and only then touching the catalog.

## Extending the set

1. New review hit → new row with the next free id; existing rows are never edited.
2. A terminology decision changes `expected_ru` → edit that row (same id) and record
   the revision with a date in the master terminology table.
3. Delete a row only when its source disappears from the catalog (lupdate dropped it).
