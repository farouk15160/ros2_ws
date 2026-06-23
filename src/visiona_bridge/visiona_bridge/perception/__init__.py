"""
Perception Package – Visiona Jarvis Pipeline.

Modules:
    object_detector_node  – VLM (LLaVA/Ollama) detects objects, emits pixel coords
    segmentation_node     – MobileSAM segments detected objects
    pose_estimator_node   – depth + TF2 → 3D world poses + RViz markers
"""
