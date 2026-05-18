# Edge Pipeline

Prompt + image -> bounding box on best matching COCO object.

No cache. Direct pipeline: YOLO26n detects all 80 COCO classes -> Model2Vec encodes prompt -> cosine vs COCO class embeddings -> rerank detections -> draw best.

## Files

- `pipeline.py` — main program
- `yolo26n.pt` — YOLO detector weights (bundled)
- `potion-base-8M/` — Model2Vec encoder (auto-downloaded on first run)
- `requirements.txt` — Python deps

## Install

```bash
pip install -r requirements.txt
```

Requires Python 3.9+. CPU-only works. GPU used if available (ultralytics auto-detects).

## Run (CLI)

```bash
python pipeline.py "a thing you sit on" path/to/image.jpg --out out.jpg
```

Args:
- `prompt` — natural language description (positional, quote it)
- `image` — input image path (positional)
- `--out` — output image path (default: `out.jpg`)
- `--conf` — YOLO confidence threshold (default: `0.05`, lower = more candidates)

First run downloads `potion-base-8M` (~30MB) into `potion-base-8M/`. Subsequent runs reuse it.

## Example

```bash
python pipeline.py "fluffy companion that barks" sample.jpg --out result.jpg --conf 0.1
```

Output (stdout):
```
prompt : 'fluffy companion that barks'
image  : C:\path\sample.jpg
conf   : 0.1
[yolo] loading C:\...\yolo26n.pt
[yolo] ready. classes=80
[m2v] loading from C:\...\potion-base-8M
[m2v] ready. coco_embs=(80, 256)
[detect] raw detections: 7
[semantic] top5 classes for prompt:
  dog                1.000
  cat                0.812
  ...
[rank] top5 detections after rerank:
  dog                score=0.945 sem=1.000 conf=0.78
  ...
best   : dog score=0.945 sem=1.000 conf=0.78
saved  : C:\path\result.jpg
```

Output image: lime box = best, yellow boxes = runners-up (top 2-6).

## Programmatic

```python
from pipeline import EdgePipeline, draw

pipe = EdgePipeline()
best, ranked = pipe.run('image.jpg', 'something to make rooms look nice')
if best:
    print(best['name'], best['score'])
    draw('image.jpg', best, ranked, 'out.jpg')
```

`EdgePipeline()` loads YOLO + M2V once; reuse the instance across many queries.

## Tuning

In `pipeline.py`:
- `ALPHA` (0.75) — semantic weight
- `BETA` (0.25) — detection confidence weight
- Combined score = `ALPHA * semantic + BETA * det_conf`

Lower `--conf` if no detections appear. Raise it for cleaner results.

## Troubleshooting

**No detections logged but no box drawn**
- YOLO found nothing above conf threshold. Lower `--conf 0.01` or check the image content.

**`FileNotFoundError: yolo26n.pt`**
- The script expects `yolo26n.pt` next to `pipeline.py`. Copy it back if removed.

**HuggingFace symlink error on Windows**
- Already handled via `local_dir_use_symlinks=False`. If it still fails, delete `potion-base-8M/` and retry.

**Output image identical to input**
- Best detection's bbox may be 0-sized or off-screen. Check `[rank]` log for actual coords.

## Architecture

```
prompt -> Model2Vec.encode -> [256-d vec]
                                  |
                                  v (cosine)
                  [a photo of a {COCO}]_80 emb -> sims_80
                                  |
                                  v normalize 0-1
                              class_scores
                                  |
image -> YOLO26n.predict -> dets[(box, conf, cls)]
                                  |
                                  v combine
              score = 0.75*class_scores[cls] + 0.25*conf
                                  |
                                  v argmax
                                best -> draw lime box
```

Latency on CPU (typical): YOLO26n ~50ms, Model2Vec encode <1ms, total ~60ms per query.
