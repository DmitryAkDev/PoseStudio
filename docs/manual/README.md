# The User Manual — how it works and how to update it

This folder IS the manual that ships inside PoseStudio (**Help → User Manual**, `F1`). Nothing here is
generated: the `.md` files are the pages, `manual.json` is the table of contents, and the build
embeds the folder into the executable. The same files read fine on GitHub.

## The rule

**A change a user can see is not finished until the manual says so.** New menu entry, new dial, new
key, new gesture, changed default, removed feature — edit the page it belongs to in the same commit.
`tools/manual/check_manual.py` (also the `manual-check` CTest test) catches what it can: it fails on a
broken manifest, a missing page, a dead link, and it warns about every menu action and viewport-strip
control whose label does not appear anywhere in the manual.

## Adding or changing a page

1. Write or edit the Markdown. Keep the dialect to what the viewer renders: headings, paragraphs,
   **bold**, *italic*, `code`, fenced code blocks, bullet and numbered lists, tables, block quotes,
   links, images. No raw HTML. Put a line of text between a fenced code block and a table that
   follows it (Qt's Markdown importer otherwise mislays the table's first cell).
2. Every page starts with one `#` title. Its `##` headings become the page's sub-entries in the
   contents tree; use `###` for finer structure that should not appear there.
3. For a **new** page, add it to `manual.json` in the place it belongs — `id` (stable: the app opens
   pages by it), `title` (as shown in the contents), `file`, and optionally `children` for pages
   grouped under it. Re-run CMake once (the folder is globbed with `CONFIGURE_DEPENDS`, so a build
   picks new files up by itself after that).
4. Run `python tools/manual/check_manual.py` and fix what it reports.

## Linking

- To another page: `[Posing](posing.md)`.
- To a section: `[the Ground button](posing.md#the-ground-button)` — the anchor is the heading's
  GitHub-style slug: lower case, spaces to hyphens, punctuation dropped (`## Joint pins (P)` →
  `#joint-pins-p`). Within the current page, `[see above](#joint-pins)`.
- To the web: a full `https://` URL; it opens in the user's browser.
- Images go in `images/` and are referenced as `![caption](images/name.png)`; the app's own icons
  are available as `![Ground](qrc:/resources/icons/ground.png)`.

## What's New

`CHANGELOG.md` at the repository root is embedded as `changelog.md` and shown as the "What's New"
page, so release notes are written once. Keep the changelog user-readable: it is read in the app.

## Writing style

Second person, present tense, short paragraphs. Say what the user does and what happens, in the
words the interface uses (menu entries and control labels in **bold**, keys in `code`). Tables for
reference material, prose for how-to. Describe what the build does today; planned features belong in
the changelog or the roadmap, not here. No vendor or product names for the figure content — the
format is described by its file extensions.

## The viewer

`src/help/helpmanual.{h,cpp}` loads the manifest and renders a page (Qt's GitHub-flavoured Markdown,
restyled for the dark theme, every heading given an anchor); `src/help/helpwindow.{h,cpp}` is the
window — contents tree, browser, search, back/forward. `resources/styles/_help.qss` styles the window.
