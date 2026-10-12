# Vendored anti-slop Oxlint plugin

Source: [dmmulroy/anti-slop](https://github.com/dmmulroy/anti-slop), commit
`c44ef22ca116d0ba62a3ff663a0bd13a3f3fa40b` (2026-09-10, "Merge pull request #36 from
K-Mistele/contrib/effect-tag-match-rules"). Upstream package identity:
`oxlint-plugin-anti-slop@0.1.2`.

Upstream path: `skills/install-anti-slop/assets/anti-slop`
(copied verbatim; the plugin also exists upstream at `src/`).

## Installed paths

- `tools/oxlint/anti-slop/index.ts` — generic plugin, registered as `anti-slop`.
- `tools/oxlint/anti-slop/effect/index.ts` — opt-in Effect plugin, copied but **not registered**
  (see "Effect plugin" below).
- `tools/oxlint/anti-slop/vendor/eslint-stylistic/` — vendored
  `padding-line-between-statements`, used by `require-readable-spacing`.

## Provenance verification

The 38 copied plugin files are byte-identical to the pinned upstream revision. Verified by
comparing Git tree hashes, where a Git tree hash pins exact paths, file modes, and blob contents.
The comparison was made on the freshly copied tree, *before* this record was added:

```
upstream  c44ef22:skills/install-anti-slop/assets/anti-slop -> a7831feb0c097943ac813ddcb1367e26eef52da5
installed tools/oxlint/anti-slop (38 copied files)          -> a7831feb0c097943ac813ddcb1367e26eef52da5
```

All 38 files match with mode `100644` on both sides. This `UPSTREAM.md` is a local addition and
is not part of the upstream tree, so the directory hash no longer equals `a7831feb`. Re-verify by
excluding this file (or by diffing against a fresh copy) rather than re-hashing the directory
as-is. The pinned commit was resolved from a recorded skill tree hash rather than assumed to be
upstream `HEAD`; it happens to also be upstream `HEAD` at install time.

Vendored sub-dependency: ESLint Stylistic commit
`435c3ea0fd26a5fef9042c4b36b6e165fbbf8d08`, per
`vendor/eslint-stylistic/UPSTREAM.md`. That file already records the local adaptations applied to
the upstream rule.

## Configuration

`.oxlintrc.json` registers the generic plugin and enables its 18 rules plus the native Oxlint
companion rule `oxc/no-accumulating-spread`, all at `"error"`. `ignorePatterns` ignores
`tools/oxlint/anti-slop/**` (vendored, not application source) alongside the standard
project-local agent-tooling directories. `@oxlint/plugins` is pinned to `1.74.0` to match the
installed `oxlint@1.74.0`; the plugin imports `eslintCompatPlugin` from it.

`no-reduce-accumulator-copy` is paired with native `oxc/no-accumulating-spread`. The custom rule
catches non-spread accumulator copies such as `Object.assign({}, acc, item)`, `Array.from(acc)`,
and accumulator `concat`/`slice` calls. Named callbacks, indirect helpers, and nested accumulator
properties are not fully analyzed, so these rules do not rule out every quadratic reducer.

`no-array-filter-map` does not infer unknown receiver types by design. Prefer lazy
`.values().filter(...).map(...).toArray()` pipelines only where the runtime supports iterator
helpers; otherwise use a single `flatMap` or a locally mutating reducer, and review callback order,
indexes, sparse arrays, `thisArg`, and filtering semantics instead of rewriting chains
mechanically.

## Effect plugin

Not registered. The project has no direct `effect` dependency in `package.json` or the lockfile
importer, and Effect rules were not requested. The source is present so a later opt-in is a
configuration change rather than a re-copy; registering it means adding
`anti-slop-effect` to `jsPlugins` and enabling `anti-slop-effect/no-manual-effect-error-tag`,
`no-manual-tag-comparison`, `no-manual-tagged-construction`, `no-service-constructor-imports`, and
`prefer-effect-match`.

That plugin covers relative project imports. Package-alias imports are not analyzed, so they are a
current limitation rather than enforced behavior.

## Intentional deviations

None in the copied source. The copy is pristine upstream, so a future update can be a true
three-way merge against `c44ef22`.

## Known gaps in this bundle

- **No tests were copied.** Upstream ships a focused test file per rule under `src/rules/` and
  `src/effect/rules/`, but this skill bundle does not include them. The vendored tree therefore has
  no test evidence of its own.
- **`vendor/eslint-stylistic/UPSTREAM.md` contains a dead reference.** It points at
  `../../rules/require-readable-spacing.test.ts` and `../../rules/require-readable-spacing-cli.test.ts`,
  neither of which exists in this copy. Read those paths as upstream-only.
- **Upstream's MIT `LICENSE` was not copied.** Upstream is MIT, Copyright (c) 2026 Dillon Mulroy.
  The bundle carries only `vendor/eslint-stylistic/LICENSE`. Restore the root license from the
  pinned revision if this directory is redistributed.

## Verification results

Installed and checked on a clean tree (`git status` clean) on branch `dashboard-rewrite`.

- Plugin loads and runs under `oxlint@1.74.0` via `pnpm exec oxlint .`.
- `pnpm run type-check` (`vue-tsc --build`) passes, exit 0.
- Oxlint exits 1 with **144 findings, all from anti-slop rules**, in owned application source:
  136 `require-readable-spacing`, 5 `require-safety-comment-for-type-assertion`,
  2 `no-known-value-widening`, 1 `no-unsafe-dictionary-type`. No `oxc/no-accumulating-spread`
  findings.
- Fixes were **not** applied: cleanup was not authorized. Lint was run without `--fix`, so no
  source file was modified. The repository's `lint` script (`oxlint . --fix`) and
  `format` script (`prettier --write`) would rewrite these files if run.

Largest finding concentrations: `src/demo.ts` (22), `src/domain/powertrain.ts` (14),
`src/domain/bms.ts` (12), `src/telemetry.ts` (11), `src/composables/useTelemetry.ts` (9).

When cleanup is authorized: apply `require-readable-spacing` with lint autofix first, then run the
formatter, then lint again, and confirm a second fix/format pass leaves files unchanged. Keep
whitespace fixes separate from semantic edits, and preserve documentation attachment and overload
groups.

## Updating

Fetch an explicit upstream revision rather than tracking `HEAD`, confirm the new
`skills/install-anti-slop/assets/anti-slop` tree hash, then diff against this copy. Since the copy
is pristine, upstream changes can be applied directly; re-verify the `@oxlint/plugins`/`oxlint`
version pair, and re-check the gaps above if the bundle has gained tests or a license.
