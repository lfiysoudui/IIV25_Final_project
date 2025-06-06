### How to run:
in lane_change_expts:
```bash 
export SUMO_HOME=/usr/share/sumo    # or where your sumo is
python run.py -c experiments/exp01.yaml     # exp01 ~ 06, mp
```


### Paper Experiment Design
- exp01: two lane scenario (8s predict horizon)
- exp02: two lane scenario with small gap to potential leader on target lane (8s predict horizon)
- exp03: two lane scenario (15s predict horizon)
- exp04: merging scenario with moederate gap (8s predict horizon)
- exp05: merging scenario with short gap (8s predict horizon)
- exp06: two lane scenario with small gap to potential leader on target lane (15s predict horizon, more uncontrolled vehicles)
- mp: 4 lanes, each with different vehicle amount flow. many vehicles, controlled : uncontrolled 0.3 : 0.7.