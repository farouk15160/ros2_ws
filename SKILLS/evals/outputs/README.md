# evals/outputs/

Paste captured model responses HERE — one file per eval name, with the
skill loaded. Used by `eval_runner.py --mode=judge` and `--parity`.

For each eval listed in `eval.yaml`, after running its prompt
(`evals/prompts/<name>.md`) against an agent that has this skill loaded,
save the full response to `evals/outputs/<name>.md`.

Then: `python3 scripts/eval_runner.py --mode=judge`

See `docs/EVAL_WORKFLOW.md` for the full procedure including the paired
`evals/outputs_baseline/` (skill OFF) used by `--parity`.
