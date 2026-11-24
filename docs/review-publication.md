# Anonymous supplementary viewer

Published review snapshot:

https://contact-transfer-review-c7f2a.engagingchina4.chatgpt.site

This is separate from the existing public GitHub Pages viewer. The research
repository remains private; its root redistribution notice is unchanged.
This document belongs in the private repository and must not be copied into the
review bundle.

The original version 1 snapshot contains 37 recordings (21 complete-task conditions, 14 matched
source-robustness conditions, two executed forced-type comparisons), the assembly
alternative-type planning rejection, both scientific comparison plots, original
meshes, all required vendor notices, and an explicit implementation-scope page.

All 37 recordings passed numerical/boolean-leaf comparison against the saved
originals. All 63 normal-viewer recordings passed the English presentation check
without modifying original reports or measurement arrays. The review integrity
manifest and 2,616 local URDF mesh references were checked. The bundled JavaScript
passed a syntax check. Explicit author/repository/local-path checks passed.

Managed browser preview failed before startup with
`listen EPERM: operation not permitted 127.0.0.1:4173`. Browser visual QA of this
snapshot is therefore **unverified**; no alternative preview server was used.
GitHub Actions run [43369031038](https://github.com/overjs94/scale-aware-contact-transfer/actions/runs/43369031038)
tests the English source at `2c26b211d414a3fa8326813ddef7ba15bf342c6f`.

The Sites provider confirmed deployment success for neutral source commit
`bcf5d806b839bbb9b31c86299b7a4ebc768054fb`, saved version 1. Source history and
credentials are not part of the served snapshot.

Rebuild with `npm --prefix web run build`, then run
`python tools/export_review_viewer.py --output <new-empty-directory> --zip <archive.zip>`.
The URL-indexed real-mesh cache at `.cache/render-qa/manifest.json` is required;
the exporter fails if any pinned asset is missing. Use the existing Sites
identity for updates, never register another Site for the same checkout.

Removing explicit identifiers cannot remove indirect linkage to already-public
matching experiments. Review data use consistently pseudonymized fingerprints;
the fresh review-integrity manifest hashes the actual distributed files.

For colleague collaboration during anonymous review, invite a repository
collaborator and use branches/pull requests. Private forks depend on repository
and account policy. Public visibility permits GitHub forks and pull requests;
MIT is a separate decision granting broader original-code reuse, modification,
and redistribution rights. No license change is needed merely to collaborate.

The margin-loading update adds 18 native recordings: 12 tall-payload trials,
two flat-panel explorations and four compact-payload explorations. The review
cohort is now 55 recordings. `robustness-comparison.html` provides synchronized
ON/OFF playback and the actual smooth-hold/repeated-pulse force profiles. See
[inside-margin-hold.md](inside-margin-hold.md) for complete results, shared
failures, certificate scope, and reproduction commands.
