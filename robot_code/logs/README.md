# Runtime Logs

`run_demo.py` and the integration notebook write timestamped logs here. `logs/*.log` is ignored so board tests do not clutter commits.

When a failure needs to be shared, force-add only that file:

```bash
git add -f logs/<log-file>.log
git commit -m "Attach board demo failure log"
git push
```

Curated historical examples live in `legacy_params/logs/`; active diagnostics belong here.
