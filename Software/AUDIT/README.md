# Software audit (2026-11-12)

Read-only audit. Nothing here changes code.

Stage 1 (Haiku): one report per area, `<area>_findings.md`: what the code
does (short) and candidate bugs, each with file:line, scenario, and a
severity guess.

Stage 2 (Sonnet): `<area>_evaluation.md`: each candidate checked against the
code (confirmed / plausible / not a bug), real severity, and suggested fix.

Areas: witness_sensing, witness_comms, iris, aperture, horizon, hitl.
