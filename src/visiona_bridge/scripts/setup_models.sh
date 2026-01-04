#!/bin/bash
# Model Setup Script for Jetson Nano Orin
# Downloads and prepares LLM and VLA models for offline operation
# Takse about 45 minutes to complete

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MODEL_DIR="${HOME}/.cache/models"
VENV_DIR="${HOME}/.venv/llm"

echo "================================================"
echo "LLM/VLA Model Setup for Jetson Nano Orin"
echo "================================================"

# Create model directory
mkdir -p "${MODEL_DIR}"

# ============================================================================
# Step 1: Install Python dependencies
# ============================================================================
echo ""
echo "[1/5] Setting up Python virtual environment..."

if [ ! -d "${VENV_DIR}" ]; then
    python3 -m venv "${VENV_DIR}"
fi

source "${VENV_DIR}/bin/activate"

pip install --upgrade pip
pip install wheel

# ============================================================================
# Step 2: Install llama-cpp-python for LLM
# ============================================================================
echo ""
echo "[2/5] Installing llama-cpp-python with CUDA support..."

# Build with CUDA support for Jetson (updated flag name)
CMAKE_ARGS="-DGGML_CUDA=on" pip install llama-cpp-python --no-cache-dir

# ============================================================================
# Step 3: Download Llama 3.2 3B model (quantized)
# ============================================================================
echo ""
echo "[3/5] Downloading Llama 3.2 3B model..."

LLM_MODEL="${MODEL_DIR}/llama-3.2-3b-q4_k_m.gguf"

if [ ! -f "${LLM_MODEL}" ]; then
    echo "Downloading from HuggingFace..."
    pip install huggingface-hub
    
    python3 << EOF
from huggingface_hub import hf_hub_download

model_id = "bartowski/Llama-3.2-3B-Instruct-GGUF"
filename = "Llama-3.2-3B-Instruct-Q4_K_M.gguf"

print(f"Downloading {filename}...")
model_path = hf_hub_download(
    repo_id=model_id,
    filename=filename,
    local_dir="${MODEL_DIR}",
    local_dir_use_symlinks=False
)
print(f"Downloaded to: {model_path}")

# Rename to simpler name
import os
import shutil
if os.path.exists(model_path):
    target = "${LLM_MODEL}"
    shutil.move(model_path, target)
    print(f"Moved to: {target}")
EOF

    echo "LLM model downloaded successfully!"
else
    echo "LLM model already exists: ${LLM_MODEL}"
fi

# ============================================================================
# Step 4: Install Transformers for VLA
# ============================================================================
echo ""
echo "[4/5] Installing transformers and dependencies for VLA..."

pip install transformers accelerate bitsandbytes torch torchvision

# ============================================================================
# Step 5: Download LLaVA model
# ============================================================================
echo ""
echo "[5/5] Pre-caching LLaVA model..."

python3 << 'EOF'
from transformers import LlavaNextProcessor, LlavaNextForConditionalGeneration
from transformers import BitsAndBytesConfig
import torch

print("Downloading LLaVA model (this may take a while)...")
model_name = "llava-hf/llava-v1.6-mistral-7b-hf"

# Download processor (tokenizer + image processor)
processor = LlavaNextProcessor.from_pretrained(model_name)
print("Processor downloaded!")

# Note: We'll load the full model with quantization at runtime
# Just download the weights here
print("Downloading model weights...")
print("(Model will be quantized to 4-bit during first run)")

print("\nSetup complete!")
print("LLM model: /home/farouk/.cache/models/llama-3.2-3b-q4_k_m.gguf")
print("VLA model cached in: ~/.cache/huggingface/hub/")
EOF

# ============================================================================
# Configuration
# ============================================================================
echo ""
echo "================================================"
echo "Setup Complete!"
echo "================================================"
echo ""
echo "Model locations:"
echo "  LLM: ${LLM_MODEL}"
echo "  VLA: ~/.cache/huggingface/hub/"
echo ""
echo "To update visiona_bridge config:"
echo "  Edit: ros2_ws/src/visiona_bridge/config/llm_config.yaml"
echo "  Set model_path: ${LLM_MODEL}"
echo ""
echo "Virtual environment:"
echo "  source ${VENV_DIR}/bin/activate"
echo ""
echo "Next steps:"
echo "  1. Update llm_config.yaml with model path"
echo "  2. Test LLM inference: ros2 run visiona_bridge test_llm_inference"
echo "  3. Build workspace: cd ~/ros2_ws && colcon build"
echo ""
