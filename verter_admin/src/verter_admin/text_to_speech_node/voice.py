# import os, subprocess, shutil

# BASE = os.path.dirname(os.path.abspath(__file__))
# MODEL = os.path.join(BASE, "ru_RU-ruslan-medium.onnx")
# CFG   = os.path.join(BASE, "ru_RU-ruslan-medium.onnx.json")
# TEXT  = "Эльвира, остановись, хватит жрать торты"

# env = os.environ.copy()
# env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"

# piper = [ "piper", "-m", MODEL, "-c", CFG, "--output-raw", "-t", TEXT ]
# aplay = [ "aplay", "-D", "pulse", "-q", "-r", "22050", "-f", "S16_LE", "-t", "raw", "-c", "1" ]

# p1 = subprocess.Popen(piper, stdout=subprocess.PIPE, env=env)
# p2 = subprocess.Popen(aplay, stdin=p1.stdout, env=env)
# p1.stdout.close()
# p2.wait();
