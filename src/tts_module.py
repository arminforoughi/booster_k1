#!/usr/bin/env python3
"""
Text-to-Speech Module for Booster K1
Supports multiple TTS engines with fallback
- Piper TTS (neural, ARM-optimized) - RECOMMENDED
- espeak (lightweight, always available)
- pyttsx3 (offline, better than espeak)
"""

import os
import threading
import queue
import subprocess
import tempfile
import time

class TextToSpeech:
    """TTS with multiple backend support and auto-fallback"""

    def __init__(self, engine='piper', piper_model=None, voice_speed=1.0):
        """
        Initialize TTS engine

        Args:
            engine: 'piper', 'espeak', 'pyttsx3', or 'auto' (auto tries in order)
            piper_model: Path to Piper model (default: auto-download lightweight model)
            voice_speed: Speed multiplier (1.0 = normal, 0.8 = slower, 1.2 = faster)
        """
        self.engine = engine
        self.voice_speed = voice_speed
        self.available = False
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.running = False

        # Piper-specific
        self.piper_binary = None
        self.piper_model_path = piper_model

        # Try engines in priority order
        if engine == 'auto':
            # Try Piper first, then espeak, then pyttsx3
            for eng in ['piper', 'espeak', 'pyttsx3']:
                if self._init_engine(eng):
                    self.engine = eng
                    break
        else:
            self._init_engine(engine)

        if not self.available:
            print("⚠️  No TTS engine available!")
            print("   Install options:")
            print("   - Piper: Download from https://github.com/rhasspy/piper/releases")
            print("   - espeak: sudo apt-get install espeak")

        # Start worker thread if available
        if self.available:
            self.start()

    def _init_engine(self, engine):
        """Initialize specific engine"""
        if engine == 'piper':
            if self._test_piper():
                print(f"✓ Using Piper TTS (model: {os.path.basename(self.piper_model_path) if self.piper_model_path else 'default'})")
                self.available = True
                return True
        elif engine == 'espeak':
            if self._test_espeak():
                print("✓ Using espeak for TTS")
                self.available = True
                return True
        elif engine == 'pyttsx3':
            if self._test_pyttsx3():
                print("✓ Using pyttsx3 for TTS")
                self.available = True
                return True
        return False

    def _test_piper(self):
        """Test if Piper TTS is available"""
        try:
            # Check for piper binary
            result = subprocess.run(['which', 'piper'], capture_output=True, text=True)
            if result.returncode == 0:
                self.piper_binary = result.stdout.strip()

                # If no model specified, try to find one
                if not self.piper_model_path:
                    self.piper_model_path = self._find_piper_model()

                # Test if model exists
                if self.piper_model_path and os.path.exists(self.piper_model_path):
                    return True
                elif self.piper_model_path:
                    print(f"⚠️  Piper model not found: {self.piper_model_path}")
                    return False
                else:
                    print("⚠️  No Piper model found. Download from: https://github.com/rhasspy/piper/releases")
                    return False
            return False
        except Exception as e:
            return False

    def _find_piper_model(self):
        """Try to find an installed Piper model"""
        # Common locations for Piper models
        search_paths = [
            '/usr/share/piper-voices',
            '/usr/local/share/piper-voices',
            os.path.expanduser('~/.local/share/piper-voices'),
            os.path.expanduser('~/piper-voices'),
            './piper-voices',
        ]

        for path in search_paths:
            if os.path.exists(path):
                # Look for .onnx files
                for root, dirs, files in os.walk(path):
                    for file in files:
                        if file.endswith('.onnx'):
                            model_path = os.path.join(root, file)
                            print(f"   Found Piper model: {model_path}")
                            return model_path
        return None

    def _test_espeak(self):
        """Test if espeak is available"""
        try:
            result = subprocess.run(['which', 'espeak'], capture_output=True)
            return result.returncode == 0
        except:
            return False

    def _test_pyttsx3(self):
        """Test if pyttsx3 is available"""
        try:
            import pyttsx3
            self.pyttsx3_engine = pyttsx3.init()
            # Set speed
            self.pyttsx3_engine.setProperty('rate', int(150 * self.voice_speed))
            return True
        except Exception as e:
            return False

    def speak(self, text, blocking=False):
        """
        Speak the given text

        Args:
            text: Text to speak
            blocking: If True, wait for speech to complete
        """
        if not self.available:
            print(f"[TTS unavailable] {text}")
            return

        if blocking:
            self._do_speak(text)
        else:
            self.speech_queue.put(text)

    def _do_speak(self, text):
        """Actually perform the speech"""
        try:
            if self.engine == 'piper':
                self._speak_piper(text)
            elif self.engine == 'espeak':
                self._speak_espeak(text)
            elif self.engine == 'pyttsx3':
                self._speak_pyttsx3(text)
        except Exception as e:
            print(f"⚠️  TTS error: {e}")

    def _speak_piper(self, text):
        """Speak using Piper TTS"""
        # Create temporary WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            wav_file = tmp.name

        try:
            # Run piper to generate audio
            # piper reads from stdin, outputs WAV to stdout
            piper_cmd = [self.piper_binary, '--model', self.piper_model_path, '--output_file', wav_file]

            # Add length scale (inverse of speed)
            if self.voice_speed != 1.0:
                piper_cmd.extend(['--length_scale', str(1.0 / self.voice_speed)])

            process = subprocess.Popen(
                piper_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Send text to piper
            process.communicate(input=text.encode('utf-8'), timeout=10)

            # Play the audio file
            if os.path.exists(wav_file) and os.path.getsize(wav_file) > 0:
                # Try aplay first (ALSA), then paplay (PulseAudio)
                play_cmd = None
                if subprocess.run(['which', 'aplay'], capture_output=True).returncode == 0:
                    play_cmd = ['aplay', '-q', wav_file]
                elif subprocess.run(['which', 'paplay'], capture_output=True).returncode == 0:
                    play_cmd = ['paplay', wav_file]

                if play_cmd:
                    subprocess.run(play_cmd, check=True, timeout=10)
                else:
                    print("⚠️  No audio player found (install alsa-utils or pulseaudio)")

        except subprocess.TimeoutExpired:
            print("⚠️  Piper TTS timeout")
            process.kill()
        except Exception as e:
            print(f"⚠️  Piper TTS error: {e}")
        finally:
            # Clean up temp file
            if os.path.exists(wav_file):
                try:
                    os.unlink(wav_file)
                except:
                    pass

    def _speak_espeak(self, text):
        """Speak using espeak"""
        # Escape single quotes
        text = text.replace("'", "'\\''")

        # Add speed control
        speed_param = int(175 * self.voice_speed)  # espeak default is 175

        os.system(f"espeak -s {speed_param} '{text}' 2>/dev/null")

    def _speak_pyttsx3(self, text):
        """Speak using pyttsx3"""
        self.pyttsx3_engine.say(text)
        self.pyttsx3_engine.runAndWait()

    def _worker(self):
        """Worker thread that processes speech queue"""
        while self.running:
            try:
                text = self.speech_queue.get(timeout=0.5)
                self._do_speak(text)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"⚠️  TTS worker error: {e}")

    def start(self):
        """Start the TTS worker thread"""
        if self.running:
            return
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()

    def stop(self):
        """Stop the TTS worker thread"""
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2.0)

    def set_speed(self, speed):
        """Set voice speed (1.0 = normal)"""
        self.voice_speed = speed
        if self.engine == 'pyttsx3' and hasattr(self, 'pyttsx3_engine'):
            self.pyttsx3_engine.setProperty('rate', int(150 * self.voice_speed))


def test_tts():
    """Test the TTS module"""
    print("\n" + "="*60)
    print("Text-to-Speech Module Test")
    print("="*60)

    # Test auto-detection
    print("\nTesting auto-detection...")
    tts = TextToSpeech(engine='auto')

    if tts.available:
        print(f"\nUsing engine: {tts.engine}")
        print("Speaking test message...")
        tts.speak("Hello, I am the Booster K1 robot. This is a test of the text to speech system.", blocking=True)

        # Test different speeds
        print("\nTesting different speeds...")
        tts.set_speed(0.8)
        tts.speak("This is slower speech.", blocking=True)

        tts.set_speed(1.2)
        tts.speak("This is faster speech.", blocking=True)

        tts.set_speed(1.0)
        tts.speak("Back to normal speed.", blocking=True)

        print("✓ TTS test complete")
    else:
        print("✗ No TTS engine available")

    tts.stop()


if __name__ == '__main__':
    test_tts()
