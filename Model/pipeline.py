import argparse
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def log(*a, **k):
    print(*a, **k, flush=True)


COCO = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat',
    'dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack',
    'umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball',
    'kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple',
    'sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair',
    'cozuch','potted plant','bed','dining table','toilet','tv','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear','hair drier',
    'toothbrush',
]
assert len(COCO) == 80

HERE = Path(__file__).resolve().parent
YOLO_WEIGHTS = HERE / 'yolo26n.pt'
M2V_DIR = HERE / 'potion-base-8M'
M2V_REPO = 'minishlab/potion-base-8M'

ALPHA = 0.75
BETA = 0.25


def _load_m2v():
    from model2vec import StaticModel
    if not (M2V_DIR / 'config.json').exists():
        from huggingface_hub import snapshot_download
        log(f'[m2v] downloading {M2V_REPO} -> {M2V_DIR}')
        snapshot_download(repo_id=M2V_REPO, local_dir=str(M2V_DIR), local_dir_use_symlinks=False)
    log(f'[m2v] loading from {M2V_DIR}')
    return StaticModel.from_pretrained(str(M2V_DIR))


def _encode(m2v, texts):
    if isinstance(texts, str):
        texts = [texts]
    v = np.asarray(m2v.encode(list(texts)), dtype=np.float32)
    n = np.linalg.norm(v, axis=-1, keepdims=True).clip(min=1e-8)
    return v / n


class EdgePipeline:
    def __init__(self):
        from ultralytics import YOLO
        if not YOLO_WEIGHTS.exists():
            raise FileNotFoundError(f'YOLO weights not found: {YOLO_WEIGHTS}')
        log(f'[yolo] loading {YOLO_WEIGHTS}')
        self.yolo = YOLO(str(YOLO_WEIGHTS))
        log(f'[yolo] ready. classes={len(self.yolo.names)}')
        self.m2v = _load_m2v()
        self.coco_embs = _encode(self.m2v, [f'a photo of a {c}' for c in COCO])
        log(f'[m2v] ready. coco_embs={self.coco_embs.shape}')
        self.pool = ThreadPoolExecutor(max_workers=2)

    def class_scores(self, prompt: str) -> np.ndarray:
        q = _encode(self.m2v, [prompt])[0]
        sims = self.coco_embs @ q
        sims = (sims - sims.min()) / (sims.max() - sims.min() + 1e-8)
        return sims.astype(np.float32)

    def detect(self, image_path, conf=0.05):
        res = self.yolo.predict(source=str(image_path), conf=conf, verbose=False)[0]
        out = []
        if res.boxes is None or len(res.boxes) == 0:
            return out
        names = res.names if hasattr(res, 'names') else {i: COCO[i] for i in range(80)}
        for b in res.boxes:
            cls = int(b.cls.item())
            out.append({
                'xyxy': b.xyxy[0].cpu().numpy().tolist(),
                'conf': float(b.conf.item()),
                'cls': cls,
                'name': names.get(cls, COCO[cls] if cls < 80 else str(cls)),
            })
        return out

    def run(self, image_path, prompt, conf=0.05):
        t0 = time.perf_counter()
        f_det = self.pool.submit(self.detect, image_path, conf)
        f_sem = self.pool.submit(self.class_scores, prompt)
        dets = f_det.result(); t_det = time.perf_counter() - t0
        sc = f_sem.result();   t_sem = time.perf_counter() - t0
        log(f'[parallel] detect={t_det*1000:.1f}ms  semantic={t_sem*1000:.1f}ms  (wallclock={max(t_det,t_sem)*1000:.1f}ms)')
        log(f'[detect] raw detections: {len(dets)}')
        if not dets:
            return None, [], []
        topk = np.argsort(-sc)[:5]
        log('[semantic] top5 classes for prompt:')
        for j in topk:
            log(f'  {COCO[j]:18s} {float(sc[j]):.3f}')
        ranked = []
        for d in dets:
            sem = float(sc[d['cls']]) if d['cls'] < 80 else 0.0
            ranked.append({**d, 'sem': sem, 'score': ALPHA * sem + BETA * d['conf']})
        ranked.sort(key=lambda x: x['score'], reverse=True)
        best = ranked[0]
        # keep every detection of same class as best (multiple instances of correct object)
        winners = [d for d in ranked if d['cls'] == best['cls']]
        log(f'[rank] best class={best["name"]!r}  winner_count={len(winners)}')
        for d in winners:
            log(f"  {d['name']:18s} score={d['score']:.3f} sem={d['sem']:.3f} conf={d['conf']:.3f}")
        return best, ranked, winners


def draw(image_path, winners, prompt, out_path, footer_h=48):
    src = Image.open(image_path).convert('RGB')
    W, H = src.size
    canvas = Image.new('RGB', (W, H + footer_h), (0, 0, 0))
    canvas.paste(src, (0, 0))
    drw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('arial.ttf', 18)
        ffont = ImageFont.truetype('arial.ttf', 20)
    except Exception:
        font = ImageFont.load_default()
        ffont = font

    for d in winners:
        x1, y1, x2, y2 = d['xyxy']
        drw.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=4)
        label = f"{d['name']} {d['score']:.2f}"
        try:
            tw = drw.textlength(label, font=font)
        except Exception:
            tw = 8 * len(label)
        drw.rectangle([x1, max(0, y1 - 22), x1 + tw + 8, y1], fill=(0, 255, 0))
        drw.text((x1 + 4, max(0, y1 - 20)), label, fill=(0, 0, 0), font=font)

    footer_text = f'prompt: {prompt}'
    try:
        tw = drw.textlength(footer_text, font=ffont)
    except Exception:
        tw = 9 * len(footer_text)
    fx = max(8, (W - tw) // 2)
    drw.text((fx, H + (footer_h - 22) // 2), footer_text, fill=(255, 255, 255), font=ffont)

    out_path = Path(out_path).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser(description='Prompt + image -> bbox on best matching COCO object.')
    ap.add_argument('prompt', help='natural language prompt, e.g. "a thing you sit on"')
    ap.add_argument('image', help='path to input image')
    ap.add_argument('--out', default='out.jpg', help='output image path (default: out.jpg)')
    ap.add_argument('--conf', type=float, default=0.05, help='YOLO confidence threshold (default 0.05)')
    args = ap.parse_args()

    img_path = Path(args.image).resolve()
    if not img_path.exists():
        log(f'ERROR: image not found: {img_path}')
        sys.exit(2)

    log(f'prompt : {args.prompt!r}')
    log(f'image  : {img_path}')
    log(f'conf   : {args.conf}')

    pipe = EdgePipeline()
    best, _, winners = pipe.run(img_path, args.prompt, conf=args.conf)
    if best is None:
        log('no detections -> nothing to draw. try lowering --conf')
        sys.exit(1)
    log(f"best   : {best['name']} score={best['score']:.3f} sem={best['sem']:.3f} conf={best['conf']:.3f}")
    out = draw(img_path, winners, args.prompt, args.out)
    log(f'saved  : {out}')


if __name__ == '__main__':
    main()
