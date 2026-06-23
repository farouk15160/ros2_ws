# Eval Workflow

`scripts/eval_runner.py` supports three operating modes. This document
explains each, the data files they read, and how to wire them into a
manual or CI workflow.

## TL;DR

| Mode | What it scores | When to run | Cost |
|------|----------------|-------------|------|
| `structural` (default) | `evals/expected/*.md` fixtures vs criteria keywords | Every CI run | 0 model calls |
| `--mode=judge` | `evals/outputs/*.md` (real model output, skill loaded) vs criteria | After capturing model output | 0 model calls (you bring the output) |
| `--parity` | Delta between `evals/outputs/*.md` and `evals/outputs_baseline/*.md` | Periodically (weekly/monthly) | 0 model calls (you bring both outputs) |

All three share the same morphology-tolerant keyword matcher (see
`scripts/eval_runner.py::_term_matches`); they differ only in which file
backs the comparison.

## Directory layout

```
evals/
  eval.yaml                # eval definitions, criteria, weights, parity config
  prompts/<name>.md        # input prompt for each eval
  expected/<name>.md       # ideal/reference answer (fixture)
  outputs/<name>.md        # USER FILL: real model output with skill loaded
  outputs_baseline/<name>.md  # USER FILL: real model output WITHOUT skill loaded
  history/<YYYY-MM>.jsonl  # parity test history (auto-generated, gitignored)
```

`outputs/`, `outputs_baseline/`, and `history/` are created on first run.
Only `outputs/` and `outputs_baseline/` should be committed when you want
to share captured baselines with collaborators.

## Mode 1 - structural (default)

```bash
python3 scripts/eval_runner.py
```

Scores `evals/expected/*.md` (the ideal answer fixtures) against each
eval's criteria. This is a structural smoke check — it catches missing
fixtures, accidentally-emptied expected files, or criteria that no
longer have any matching content. It does NOT score model output.

This is the CI gate. Default thresholds (per-criterion coverage 0.30,
overall pass rate 80%) are deliberately permissive.

### When it fails

- Expected file deleted or emptied → "Empty expected file" error.
- Criteria reworded with no keyword overlap to expected → coverage drops
  below 0.30. Either revise the criterion or extend the expected text.
- Adjust strictness: `--min-coverage 0.5 --min-pass-rate 90`.

## Mode 2 - judge (real model output)

```bash
# 1. Open Claude (or whatever agent) WITH the skill loaded.
# 2. Paste each evals/prompts/<name>.md into the agent.
# 3. Save the model's full response to evals/outputs/<name>.md
# 4. Score it:
python3 scripts/eval_runner.py --mode=judge
```

This scores the actual model output you captured. Useful for:

- Did the model with the skill loaded actually meet the criteria?
- Did a recent skill change break a real-world answer?

Missing capture files surface as `[SKIP]`, not `[FAIL]`. The overall
status reports `[NODATA]` if nothing was captured; the exit code stays 0
so judge mode can sit safely in CI even when outputs are partially
populated.

## Mode 3 - parity (skill ON vs OFF)

```bash
# 1. Open a fresh session WITHOUT the skill loaded.
# 2. Paste each prompt and save the response to evals/outputs_baseline/<name>.md
# 3. Open a session WITH the skill loaded.
# 4. Paste the same prompts and save responses to evals/outputs/<name>.md
# 5. Run parity:
python3 scripts/eval_runner.py --parity
```

For each eval, parity scores both the ON and OFF capture, then reports
`delta = on_pass_rate - off_pass_rate`. The aggregate average delta is
compared against `parity_test.threshold` in `eval.yaml` (default 5.0%).

Every run appends one JSON-lines entry to
`evals/history/<YYYY-MM>.jsonl`. If the most recent
`consecutive_failures_for_deprecation` runs (default 3) all sit below
threshold, the report prints:

```
*** DEPRECATION CANDIDATE: most recent 3 runs all below threshold ***
```

and the process exits with code 2.

### Why parity matters

Without it, you have no way to tell whether the skill is helping
anymore. Models change. The skill's value-add might shrink over time as
the base model gets better. The threshold-with-streak mechanic gives you
an early warning rather than a gradual silent erosion.

## CLI cheat sheet

```bash
# Structural smoke check (default; CI gate)
eval_runner.py
eval_runner.py --min-coverage 0.5 --min-pass-rate 90

# Score real model output you've captured
eval_runner.py --mode=judge
eval_runner.py --mode=judge --eval-name qos-compatibility-analysis

# Parity: ON vs OFF with deprecation streak check
eval_runner.py --parity

# Common
eval_runner.py --json           # machine-readable output
eval_runner.py --verbose        # extra fields (paths, lengths)
```

## Recommended cadence

| Trigger | Run |
|---------|-----|
| Every PR | `eval_runner.py` (structural; in CI) |
| Each release candidate | Refresh `evals/outputs/` + `eval_runner.py --mode=judge` |
| Monthly | Refresh both `outputs/` + `outputs_baseline/` + `eval_runner.py --parity` |

The structural gate is cheap and catches fixture rot in CI. Judge and
parity need human capture and are too expensive for every PR.

## FAQ

**Q: Do I have to capture all 11 outputs to run parity?**
No. Capture the ones you care about. Evals with missing outputs are
marked `[SKIP]` and excluded from the aggregate delta.

**Q: How do I add an LLM-as-judge?**
Out of scope for this round. The keyword matcher with prefix matching
catches morphological variants (paths/path, warnings/warn) which was
the main practical gap. A real LLM judge would call the API per
criterion — non-trivial to integrate and costs real money per run.
Tracked as future work in `eval.yaml`.

**Q: Why JSON-lines for history?**
Append-only, parses incrementally, survives partial writes. Easy to
graph with `jq` or pandas.
