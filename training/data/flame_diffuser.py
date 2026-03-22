#!/usr/bin/env python3
"""FLAME Diffuser — Stable Diffusion 기반 화재 합성 이미지 생성.

SD v1.5 img2img 파이프라인으로 기존 화재 이미지의 조명·연기·환경을 다양화.
참조: FLAME Diffuser (arXiv 2024) 방법론 적용.

사용법:
  python flame_diffuser.py --input synfire_yolo/images --output flame_output --count 100
  python flame_diffuser.py --count 1000 --strength 0.5   # RTX 4050: ~2초/장

요구사항:
  - diffusers, transformers, accelerate
  - VRAM 6GB+ (fp16 모드)
  - SD v1.5 자동 다운로드 (~4GB, 최초 1회)

라이선스:
  - Stable Diffusion v1.5: CreativeML Open RAIL-M (비상업 제한 없음, 해로운 용도 금지)
  - 생성 이미지: ARGOS 학습 데이터용 (Apache 2.0 메인과 분리)
"""
import argparse
import os
import random
from pathlib import Path

import torch
from PIL import Image


# 화재 장면 다양화 프롬프트
FIRE_PROMPTS = [
    "indoor fire scene with thick black smoke, emergency lighting, realistic",
    "warehouse fire with flames spreading on floor, smoke filling ceiling",
    "office fire, burning furniture, orange flames, heavy smoke",
    "kitchen fire, grease fire on stove, smoke alarm, realistic lighting",
    "electrical fire with sparks, blue-white flames, dark room",
    "industrial fire in factory, large flames, metal structures",
    "corridor fire, smoke layer at ceiling, emergency exit signs glowing",
    "basement fire, low visibility, flashlight beam through smoke",
    "building fire at night, flames visible through windows, smoke rising",
    "car fire in garage, intense orange flames, melting materials",
]

NEGATIVE_PROMPT = (
    "cartoon, anime, drawing, painting, illustration, "
    "low quality, blurry, watermark, text, logo"
)


def load_pipeline(device="cuda"):
    """SD v1.5 img2img 파이프라인 로드 (fp16)."""
    from diffusers import StableDiffusionImg2ImgPipeline

    print("[FLAME] Loading Stable Diffusion v1.5 (fp16)...")
    pipe = StableDiffusionImg2ImgPipeline.from_pretrained(
        "stable-diffusion-v1-5/stable-diffusion-v1-5",
        torch_dtype=torch.float16,
        safety_checker=None,  # 화재 이미지가 safety checker에 걸릴 수 있음
    )
    pipe = pipe.to(device)
    pipe.enable_attention_slicing()  # VRAM 절약 (6GB 대응)
    print(f"[FLAME] Pipeline loaded on {device}")
    return pipe


def generate_variations(
    pipe,
    input_dir: Path,
    output_dir: Path,
    count: int = 100,
    strength: float = 0.5,
    guidance_scale: float = 7.5,
    seed: int = 42,
):
    """입력 이미지를 기반으로 다양한 화재 장면 생성.

    Args:
        strength: 0.3(원본 유지) ~ 0.7(크게 변형). 0.5 권장.
        guidance_scale: 프롬프트 반영 강도. 7.5 기본.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # 입력 이미지 목록
    input_images = sorted(input_dir.glob("*.png")) + sorted(input_dir.glob("*.jpg"))
    if not input_images:
        print(f"[오류] 입력 이미지 없음: {input_dir}")
        return

    print(f"[FLAME] 입력: {len(input_images)}장, 생성 목표: {count}장")
    print(f"[FLAME] strength={strength}, guidance={guidance_scale}")

    generator = torch.Generator(device="cuda").manual_seed(seed)
    generated = 0

    for i in range(count):
        # 랜덤 입력 이미지 선택
        src_img = Image.open(random.choice(input_images)).convert("RGB")
        src_img = src_img.resize((512, 512))  # SD v1.5 기본 해상도

        # 랜덤 프롬프트 선택
        prompt = random.choice(FIRE_PROMPTS)

        # 생성
        result = pipe(
            prompt=prompt,
            negative_prompt=NEGATIVE_PROMPT,
            image=src_img,
            strength=strength,
            guidance_scale=guidance_scale,
            num_inference_steps=30,
            generator=generator,
        )

        # 저장
        out_path = output_dir / f"flame_{i:04d}.png"
        result.images[0].save(out_path)
        generated += 1

        if generated % 10 == 0:
            print(f"  [{generated}/{count}] {out_path.name}")

    print(f"\n[FLAME] 완료: {generated}장 → {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="FLAME Diffuser 화재 합성 이미지 생성")
    parser.add_argument("--input", default=None,
                        help="입력 이미지 디렉토리 (없으면 텍스트 프롬프트만 사용)")
    parser.add_argument("--output", default="training/data/flame_output",
                        help="출력 디렉토리")
    parser.add_argument("--count", type=int, default=100,
                        help="생성할 이미지 수 (기본: 100)")
    parser.add_argument("--strength", type=float, default=0.5,
                        help="변형 강도 0.3~0.7 (기본: 0.5)")
    parser.add_argument("--seed", type=int, default=42,
                        help="랜덤 시드")
    parser.add_argument("--device", default="cuda",
                        help="디바이스 (기본: cuda)")
    args = parser.parse_args()

    # 입력 디렉토리 자동 탐지
    if args.input is None:
        candidates = [
            Path("training/data/synfire_yolo/images"),
            Path("training/data/fire_dataset/train/images"),
        ]
        for c in candidates:
            if c.exists():
                args.input = str(c)
                break
        if args.input is None:
            print("[오류] 입력 이미지를 찾을 수 없습니다. --input 지정 필요")
            return

    pipe = load_pipeline(args.device)
    generate_variations(
        pipe,
        input_dir=Path(args.input),
        output_dir=Path(args.output),
        count=args.count,
        strength=args.strength,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
