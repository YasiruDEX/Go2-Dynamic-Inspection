from typing import Optional, List
import random
import time
import os
import json
import glob
from loguru import logger
from registry import registry, GeneratorMeta
from artifacts import Artifact
import generators  # noqa
import argparse
import os.path as osp
from question_template import get_question_template

generators.autodiscover()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--tag", type=str, default=None, help="filter generators by tag"
    )
    parser.add_argument(
        "--num", type=int, default=10, help="number of images for each generator"
    )
    parser.add_argument("--output", type=str, default="output")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "-g",
        "--generators",
        type=str,
        nargs="+",
        default=None,
        help="filter generators by name",
    )
    parser.add_argument(
        "-l",
        "--list",
        action="store_true",
        help="list all registered generators",
    )
    parser.add_argument(
        "-r", "--resume", action="store_true", help="resume the last generation"
    )
    return parser.parse_args()


def select_generator(tag: str, rng: random.Random) -> GeneratorMeta:
    metas = registry.list(include_tags={tag})
    logger.info(f"Selected generators: {metas}")
    return registry.weighted_choice(metas, rng)


def run_once(img_path: str, generator: GeneratorMeta) -> Artifact:
    artifact = generator.func(img_path)
    artifact.generator = generator.name
    logger.info(f"Selected artifact: {artifact}")
    return artifact


def build_annotation(artifact: Artifact, question_id: str, rng: random.Random):
    if artifact.question is None:
        artifact.question = get_question_template(artifact, rng)
    annotation = {
        "question_id": f"{question_id}",
        "question": artifact.question,
        "img_path": artifact.data,
        "image_type": artifact.image_type,
        "question_type": "open",
        "design": artifact.design,
        "evaluator": artifact.evaluator,
        "evaluator_kwargs": artifact.evaluator_kwargs,
        "meta_info": {"source": artifact.generator, "uploader": "", "license": ""},
    }
    return annotation


def remove_generated_metas(
    metas: List[GeneratorMeta], output: str
) -> List[GeneratorMeta]:
    json_files = glob.glob(osp.join(output, "*.json"))
    generated_set = {
        osp.splitext(osp.basename(json_path))[0] for json_path in json_files
    }
    filterd_metas = []
    for meta in metas:
        if meta.name not in generated_set:
            filterd_metas.append(meta)
    return filterd_metas


def generate_data(
    num: int,
    output: str,
    tag: Optional[str] = None,
    seed: Optional[int] = None,
    generators: Optional[List[str]] = None,
    is_resume: Optional[bool] = False,
):
    rng = random.Random(seed if seed is not None else time.time_ns())
    if generators is None:
        # use all generators
        if tag:
            metas = registry.list(include_tags={tag})
        else:
            metas = registry.list()
    else:
        metas = []
        for generator in generators:
            metas.append(registry.get(generator))

    if is_resume:
        metas = remove_generated_metas(metas, output)

    for i in range(len(metas)):
        annotations = []
        output_img_dir = osp.join(output, metas[i].name)
        os.makedirs(output_img_dir, exist_ok=True)
        for j in range(num):
            question_id = f"{metas[i].name}_{j}"
            img_path = osp.join(output_img_dir, f"{question_id}.jpg")
            artifact = run_once(img_path, metas[i])
            artifact.data = osp.relpath(artifact.data, output)
            annotations.append(build_annotation(artifact, question_id, rng))
        with open(
            osp.join(output, f"{metas[i].name}.json"), "w", encoding="utf-8"
        ) as f:
            json.dump(annotations, f, indent=4, ensure_ascii=False)
    logger.info(f"Generated {num * len(metas)} images for {len(metas)} generators")


def show_registry_names():
    generator_names = []
    tags = set()
    for generator in registry.list():
        generator_names.append(generator.name)
        for tag in generator.tags:
            tags.add(tag)
    print("Available generators:")
    print(generator_names)
    print(f"Available tags: {tags}")


if __name__ == "__main__":
    args = parse_args()
    if args.list:
        show_registry_names()
        exit()
    generate_data(
        num=args.num,
        output=args.output,
        tag=args.tag,
        seed=args.seed,
        generators=args.generators,
        is_resume=args.resume,
    )
