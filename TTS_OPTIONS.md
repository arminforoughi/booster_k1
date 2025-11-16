# TTS Options for Booster K1

## Current Implementation

The system uses **auto-detection** and tries engines in this order:
1. **Piper TTS** (if installed) - Best quality
2. **espeak** (if installed) - Lightweight fallback
3. **pyttsx3** (if installed) - Alternative fallback

## Comparison

### Piper TTS ✅ RECOMMENDED

**Pros:**
- Neural TTS - natural sounding voice
- Optimized for ARM/Raspberry Pi/Jetson
- Fast inference on ARM devices
- Offline - no internet required
- Multiple voice models available
- Low resource usage

**Cons:**
- Requires manual installation
- ~50MB download (binary + voice model)

**Installation:**
```bash
bash install_piper.sh
```

**Use case:** Production use on K1 robot

---

### espeak (Current fallback)

**Pros:**
- Lightweight (~1MB)
- Available in most Linux repos
- Very fast
- Offline

**Cons:**
- Robotic voice quality
- Limited expressiveness

**Installation:**
```bash
sudo apt-get install espeak
```

**Use case:** Fallback when Piper unavailable

---

### pyttsx3 (Tested, had ARM dependency issues)

**Pros:**
- Python library
- Offline
- Cross-platform

**Cons:**
- Dependency issues on ARM/Jetson
- Uses espeak backend on Linux anyway
- No real advantage over direct espeak

**Status:** Not recommended due to dependency issues

---

### F5-TTS (Tested, failed)

**Pros:**
- State-of-the-art neural TTS
- Excellent voice quality

**Cons:**
- Very heavy (~2GB+ models)
- Slow on Jetson
- Complex dependencies
- Requires CUDA for decent speed
- Over-engineered for this use case

**Status:** Abandoned - too heavy for embedded use

---

## Installation Priority

1. **Try Piper first** (best balance)
   ```bash
   bash install_piper.sh
   ```

2. **Fall back to espeak** (if Piper fails)
   ```bash
   sudo apt-get install espeak
   ```

3. **System auto-detects** - no code changes needed!

---

## Changing TTS Engine

The system automatically uses the best available engine. To force a specific engine:

```python
# In smart_recognition.py or tts_module.py
tts = TextToSpeech(engine='piper')   # Force Piper
tts = TextToSpeech(engine='espeak')  # Force espeak
tts = TextToSpeech(engine='auto')    # Auto-detect (default)
```

---

## Voice Quality Comparison

| Engine | Quality | Speed | Size | ARM-friendly |
|--------|---------|-------|------|--------------|
| Piper | ★★★★★ | ★★★★☆ | ~50MB | ✅ Yes |
| espeak | ★★☆☆☆ | ★★★★★ | ~1MB | ✅ Yes |
| pyttsx3 | ★★☆☆☆ | ★★★★☆ | ~5MB | ⚠️ Issues |
| F5-TTS | ★★★★★ | ★★☆☆☆ | ~2GB | ❌ Too slow |

---

**Recommendation:** Install Piper TTS for the best experience!
