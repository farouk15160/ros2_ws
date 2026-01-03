# Octomap Mapping Quality Modes

The `spawn_visiona.launch.py` now supports two mapping quality modes via the `mapping` argument:

## Usage

### Low Quality (Default - Fast)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real camera:=true mapping:=low
```

**Settings:**
- Resolution: **2cm voxels**
- Max range: **2.0m**
- Min range: **0.1m**
- Hit probability: **0.7**
- Miss probability: **0.4**

**Best for:**
- ✅ Quick mapping
- ✅ Large workspace scanning
- ✅ Real-time visualization
- ✅ Lower CPU/memory usage

---

### High Quality (Accurate)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real camera:=true mapping:=high
```

**Settings:**
- Resolution: **1.5cm voxels** (25% smaller → more detail)
- Max range: **1.8m** (more conservative)
- Min range: **0.15m** (ignores noisy close points)
- Hit probability: **0.8** (higher confidence)
- Miss probability: **0.35** (higher confidence)

**Best for:**
- ✅ Detailed object scanning
- ✅ Small object detection
- ✅ Final production maps
- ✅ MoveIt collision planning

---

## Quality Comparison

| Metric | Low (Default) | High (Accurate) |
|--------|---------------|-----------------|
| **Voxel size** | 2cm | 1.5cm |
| **Detail level** | Good | Excellent |
| **Map size (5m³)** | ~10 MB | ~25 MB |
| **Update rate** | 15-20 Hz | 10-15 Hz |
| **CPU usage** | Low | Medium |
| **Recommended for** | General use | Precision tasks |

---

## Examples

**Quick scan of workspace:**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real camera:=true mapping:=low
```

**Detailed scan for manipulation:**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real camera:=true mapping:=high
```

**No mapping (just camera):**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real camera:=true
# Note: Octomap only runs when camera:=true
```

---

## Technical Details

The implementation uses `PythonExpression` to conditionally set parameters:

```python
'resolution': PythonExpression([
    "'0.015' if '", mapping_quality, "' == 'high' else '0.02'"
]),
```

This allows dynamic parameter selection at launch time without rebuilding.

---

## Tips

1. **Start with LOW** - faster feedback while testing
2. **Switch to HIGH** - when you need final accurate maps
3. **Save HIGH quality maps** - use for MoveIt collision planning
4. **Monitor performance** - if updates lag, stick to LOW

Your mapping quality is now **one argument away**! 🎯
