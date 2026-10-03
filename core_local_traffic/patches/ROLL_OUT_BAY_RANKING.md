# Rollout bay-ranking continuation

The repository's `feat/local-traffic-replan` branch keeps the evaluator and
reproduction harness. The complete RMF traffic source changes are exported as
compressed `git format-patch` streams because the working RMF traffic clones
only have the upstream Open-RMF remote configured.

- `rmf_traffic-3.3-rollout-bay-ranking.patch.gz`: commit `4cb4fdc`, based on
  RMF traffic 3.3.3 (`ab881a6`).
- `rmf_traffic-3.8-rollout-bay-ranking-and-online-results.patch.gz`: commits
  `30a3b53` and `e34bc6a`, based on tag `3.8.0`; includes the ABI-compatible
  online RMF/VDA5050 simulation logs and reproduction files.

Apply either stream to the matching base with:

```bash
gzip -dc PATCH.patch.gz | git am
```

The 3.8 online two-robot replan run completed both original tasks through bay
6137 with a 3.072 m minimum sampled center distance. The staged C/D run is
recorded as unresolved because PARK_E occupancy and non-atomic group command
application left B1/C mutually blocked; it is not reported as a successful
four-robot deadlock resolution.
