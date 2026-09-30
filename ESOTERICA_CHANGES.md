# Esoterica Box3D changes

This branch imports the current `src/` and `include/` trees from
`Code/Engine/ThirdParty/box3d` in Esoterica on 2026-09-30. Its upstream base is
`47d7f7cc7e091142c08d11dc7d2e493c5d34f536`, the runtime pin recorded by Esoterica.
The import preserves the source bytes, including uncommitted vendor changes.

Seven files differ from that base. They add kinematic mesh contacts, bounded
relative-motion impact queries, moving-mesh neighbor wake handling, mesh cache
updates, and Esoterica's always-enabled assertions and DLL export/import settings.
The impact query is opt-in and bounded; dynamic meshes remain unsupported.

The remaining upstream build, test, sample, and documentation files come from
the matching base commit. Esoterica's historical docs/samples/shared copies and
Engine adapters are not imported into this standalone library repository.
Esoterica itself is unchanged and still builds its vendored library.

This branch intentionally retains the runtime pin instead of replacing the
fork's newer main branch. Updating to newer upstream code requires a separate
merge and validation of these patches.

Transfer validation: all 98 source/header files matched Esoterica byte-for-byte
before committing. The matching upstream unit test executable built and passed
in both float and double precision with Visual Studio 2026, x64 Release.
These are standalone library checks, not a new Engine or moving-deck acceptance run.
