# amr_sweeper_default_missions

Ships the built-in manual mission templates that the mission executor can expose without treating
`/missions` as a source-of-truth catalog.

The runtime `/missions` directory is reserved for:
- schedule files
- per-mission execution history under `/missions/<mission_id>/<execution_timestamp>/...`
