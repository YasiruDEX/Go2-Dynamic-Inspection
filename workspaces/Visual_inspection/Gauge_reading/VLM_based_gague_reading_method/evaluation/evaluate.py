import json
from datasets import load_dataset
from evaluation.measure_bench_evaluator import MeasureBenchEvaluator
import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-file", type=str, required=True)
    parser.add_argument("--split", type=str, required=True)
    parser.add_argument("--output-dir", type=str, default="eval_results")
    return parser.parse_args()


def get_annotations(dataset):
    annotations = {}
    for d in dataset:
        annotation = {
            "question_id": d["question_id"],
            "question": d["question"],
            "image_type": d["image_type"],
            "design": d["design"],
            "evaluator": d["evaluator"],
            "evaluator_kwargs": json.loads(d["evaluator_kwargs"]),
        }
        annotations[d["question_id"]] = annotation
    return annotations


if __name__ == "__main__":
    args = parse_args()

    mb_evaluator = MeasureBenchEvaluator(
        tracker_type="image_type", tracker_subtype="design"
    )
    dataset = load_dataset("philokey/MeasureBench", split=args.split)
    predictions = json.load(open(args.result_file))
    annotations = get_annotations(dataset)
    for pred in predictions:
        question_id = pred["question_id"]
    result_name = args.result_file.split("/")[-1].split(".")[0]
    mb_evaluator.process(
        result_name=result_name,
        predictions=predictions,
        annotations=annotations,
        output_dir=args.output_dir,
    )
