# Gold sets — translation regression checks

A gold set pins source strings where the model or the prompt has already failed,
plus explicit terminology decisions. A translation that differs from `expected_ru`
is a regression: it appears as an M1 hit in the review report, not a silent change.

## Files

| File | Purpose |
|---|---|
| `ru.gold.tsv` | Russian gold set, four TAB-separated columns: `id`, `source`, `expected_ru`, `category` |
| `manual_ru.gold.tsv` | Russian manual gold set (terminology hits, not prose), same four columns; ids `GS-M-NNN` |

## Format

- `id` — stable (`GS-NNN`). The set grows by adding rows only; ids are never reused.
- `source` / `expected_ru` — the strings exactly as stored in the `.ts` catalog:
  HTML entities as written in the file (`couldn&apos;t`), embedded tabs escaped as
  two-character `\t` so the TSV columns stay intact (this matches the escaping
  `dump.py` prints, one for one).
- `category` — the term or rule class the row guards (elevation, release, pin, …).

## Checking a catalog

`gold_check.py <catalog.ts>` reads the gold set (default `gold/<lang>.gold.tsv`,
language inferred from the catalog name) and compares every row against the
catalog after normalizing both sides (HTML entities unescaped, the two-character
\t of the TSV read back as a real tab). It reports rows whose source is missing
from the catalog and rows whose translation differs from `expected_ru`, and exits
1 on any hit. The set is a check on the pipeline (model + prompt), not a queue of
edits: fixing a hit means fixing the prompt or the table, re-running, and only
then touching the catalog.

## Manual gold set

`manual_ru.gold.tsv` guards the user-manual tree (`docs/i18n/ru/manual/`) instead of a .ts catalog: a row pins an EN term (or form) and the RU form the translation must use. It is checked by `tools/i18n/manual/audit_manual.py`, which searches the RU tree for the expected form (after the same normalization as the catalog check) — a missing form is a hit, reported with the id and the expected vs actual text.

Growth rules:

1. Terminology hits only, never prose: a row pins a term or a fixed form where the model erred or a decision was made explicitly (as in the UI set). Exact prose lines are not pinned — they go stale with every EN edit.
2. New review hit → new row with the next free `GS-M-NNN` id; existing rows are never edited, and a changed decision edits its own row (same id) with the date recorded in the terminology table.
3. Delete a row only when its EN term no longer occurs anywhere in the English manual.
## Extending the set

1. New review hit → new row with the next free id; existing rows are never edited.
2. A terminology decision changes `expected_ru` → edit that row (same id) and record
   the revision with a date in the master terminology table.
3. Delete a row only when its source disappears from the catalog (lupdate dropped it).

## Bootstrapping a new language

A gold set is built from the first reviewed run, not written in advance:

1. Run the pipeline on the empty catalog with the shipped prompt and model config.
2. Read the dump through (the human pass). Every string you correct — or where the
   model's choice was wrong but you kept it — is a candidate row: record the source,
   the expected translation and the term/rule class it guards.
3. Verify disputed terms against the established reference for that language and
   domain before recording them; the master terminology table is the single source of
   `expected` values, the prompt table a subset of it.
4. Add the rows with ids starting at `<LANG>-001`, re-run the cleared strings through
   the pipeline, and require an exact match on every row before the catalog ships.
