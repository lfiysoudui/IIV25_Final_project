in lane change expts:
```bash 
export SUMO_HOME=/usr/share/sumo
python run.py -c experiments/exp01.yaml
```
- exp01: two lane scenraio (8s predict horizon)
- exp02: two lane scenario with small gap to potential leader on target lane (8s predict horizon)
- exp03: two lane scenario (15s predict horizon)
- exp04: merging scenario with moederate gap (8s predict horizon)
- exp05: merging scenario with short gap (8s predict horizon)