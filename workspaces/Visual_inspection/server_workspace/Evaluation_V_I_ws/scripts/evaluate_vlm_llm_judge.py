#!/usr/bin/env python3
"""
evaluate_vlm_llm_judge.py
Run on LAPTOP. Requires Google Gemini API key.

Uses Gemini 1.5 Pro as a judge to evaluate whether the VLM's
inspection report is CORRECT / PARTIALLY_CORRECT / INCORRECT
compared to a human-written ground truth caption.

Usage:
  export GEMINI_API_KEY=your_key_here
  python3 scripts/evaluate_vlm_llm_judge.py \
      --results results/vlm_eval_results.csv \
      --images  eval_dataset/vlm_eval/ \
      --out     results/llm_judge_results.csv
"""

import argparse, csv, json, os, sys, time
from pathlib import Path

def judge_one(judge_model, image_path, caption, vlm_output):
    import google.generativeai as genai
    from PIL import Image

    img = Image.open(image_path)
    prompt = f"""You are an expert evaluator for an industrial visual inspection AI system.

GROUND TRUTH (what a human expert says about this image):
"{caption}"

VLM INSPECTION REPORT (what the AI system produced):
Decision:   {vlm_output.get('predicted', 'N/A')}
Expected:   {vlm_output.get('expected', 'N/A')}
Confidence: {vlm_output.get('confidence', 'N/A')}
Summary:    {vlm_output.get('summary', 'N/A')}

Your task:
1. Look at the image
2. Compare the AI report against the ground truth caption
3. Rate: CORRECT / PARTIALLY_CORRECT / INCORRECT
4. Explain specifically what is right and what is wrong

Respond ONLY in valid JSON (no markdown):
{{"rating": "CORRECT|PARTIALLY_CORRECT|INCORRECT", "explanation": "...", "decision_correct": true|false, "findings_accurate": true|false}}
"""
    resp = judge_model.generate_content([prompt, img])
    text = resp.text.strip()
    # Strip markdown code block if present
    if text.startswith('```'):
        text = text.split('```')[1]
        if text.startswith('json'):
            text = text[4:]
    return json.loads(text.strip())

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', required=True, help='vlm_eval_results.csv from evaluate_vlm.py')
    ap.add_argument('--images',  required=True, help='Base folder: eval_dataset/vlm_eval/')
    ap.add_argument('--out',     default='results/llm_judge_results.csv')
    args = ap.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: Set GEMINI_API_KEY environment variable')
        sys.exit(1)

    import google.generativeai as genai
    genai.configure(api_key=api_key)
    judge = genai.GenerativeModel('gemini-1.5-pro')

    results_path = Path(args.results)
    images_base  = Path(args.images)
    out_path     = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(results_path, newline='') as f:
        rows = list(csv.DictReader(f))

    # Only judge rows that have a caption
    to_judge = [r for r in rows if r.get('caption', '').strip()]
    print(f'\nLLM Judge: {len(to_judge)}/{len(rows)} rows have captions to judge')

    judge_results = []
    for i, row in enumerate(to_judge):
        folder   = row['folder']
        fname    = row['filename']
        caption  = row['caption']
        img_path = images_base / folder / fname

        if not img_path.exists():
            print(f'  SKIP {fname} — image not found at {img_path}')
            continue

        print(f'  [{i+1}/{len(to_judge)}] Judging {fname}...', end='', flush=True)
        try:
            verdict = judge_one(judge, img_path, caption, row)
            rating  = verdict.get('rating', 'ERROR')
            print(f' {rating}')
            judge_results.append({
                'folder': folder, 'filename': fname,
                'object_type': row['object_type'],
                'expected': row['expected'], 'predicted': row['predicted'],
                'caption': caption,
                'llm_rating': rating,
                'decision_correct': verdict.get('decision_correct'),
                'findings_accurate': verdict.get('findings_accurate'),
                'explanation': verdict.get('explanation', '')[:200]
            })
            time.sleep(1)  # Rate limit
        except Exception as e:
            print(f' ERROR: {e}')
            judge_results.append({
                'folder': folder, 'filename': fname,
                'object_type': row['object_type'],
                'expected': row['expected'], 'predicted': row['predicted'],
                'caption': caption, 'llm_rating': 'ERROR',
                'decision_correct': None, 'findings_accurate': None,
                'explanation': str(e)
            })

    if judge_results:
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=judge_results[0].keys())
            writer.writeheader()
            writer.writerows(judge_results)

        # Summary
        from collections import Counter
        counts = Counter(r['llm_rating'] for r in judge_results)
        total  = len(judge_results)
        print(f'\n=== LLM JUDGE SUMMARY ===')
        print(f'CORRECT:           {counts["CORRECT"]}/{total} ({counts["CORRECT"]/total*100:.0f}%)')
        print(f'PARTIALLY_CORRECT: {counts["PARTIALLY_CORRECT"]}/{total} ({counts["PARTIALLY_CORRECT"]/total*100:.0f}%)')
        print(f'INCORRECT:         {counts["INCORRECT"]}/{total} ({counts["INCORRECT"]/total*100:.0f}%)')
        print(f'Results saved to:  {out_path}')

if __name__ == '__main__':
    main()
