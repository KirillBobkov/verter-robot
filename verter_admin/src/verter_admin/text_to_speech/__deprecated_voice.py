import os
import subprocess
import tempfile

BASE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE, "ru_RU-ruslan-medium.onnx")  # можно заменить на dmitri-модель
CFG   = os.path.join(BASE, "ru_RU-ruslan-medium.onnx.json")
TEXT  = "Здравствуйте! Проверка начала фразы."

env = os.environ.copy()
env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
env["LC_ALL"] = "C.UTF-8"
env["LANG"] = "C.UTF-8"

with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp:
    WAV = tmp.name

# Синтез в WAV, текст подаем в stdin (без -t)
piper_cmd = ["piper", "--model", MODEL, "-c", CFG, "-w", WAV]
p1 = subprocess.Popen(piper_cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
p1.stdin.write((TEXT.strip() + "\n").encode("utf-8"))
p1.stdin.close()
p1.wait()

# Воспроизведение WAV
subprocess.run(["aplay", "-D", "pulse", "-q", WAV], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# Удаление временного файла
try:
    os.unlink(WAV)
except Exception:
    pass