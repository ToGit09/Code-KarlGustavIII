import shutil
import os
import re

SRC_DIR = "src"
TARGET_DIR = "releases"

os.makedirs(TARGET_DIR, exist_ok=True)

pattern = re.compile(r"Karl-GustavIII-v(\d+)\.(\d+)\.(\d+)\.cpp$")

source_file = None
major = minor = bugfix = None

# Datei im src-Ordner suchen
for filename in os.listdir(SRC_DIR):
    match = pattern.match(filename)
    if match:
        source_file = filename
        major, minor, bugfix = map(int, match.groups())
        break

if not source_file:
    print("Keine Datei im Format programmTestvX.Y.Z.cpp im src-Ordner gefunden.")
    exit(1)

print(f"Gefundene Datei: {source_file}")

print("Welche Art von Update möchtest du durchführen:")
print("b oder Enter  - Bugfix")
print("m oder min    - Minor")
print("M oder maj    - Major")

choice = input("Auswahl: ").strip()

# Neue Version berechnen
new_major, new_minor, new_bugfix = major, minor, bugfix

if choice in ["", "b"]:
    new_bugfix += 1
elif choice in ["m", "min"]:
    new_minor += 1
    new_bugfix = 0
elif choice in ["M", "maj"]:
    new_major += 1
    new_minor = 0
    new_bugfix = 0
else:
    print("Ungültige Eingabe.")
    exit(1)

# Dateinamen
old_filename = source_file
new_filename = f"Karl-GustavIII-v{new_major}.{new_minor}.{new_bugfix}.cpp"

print(f"Alte Datei: {old_filename}")
print(f"Neue Datei: {new_filename}")

src_old = os.path.join(SRC_DIR, old_filename)
src_new = os.path.join(SRC_DIR, new_filename)
release_copy = os.path.join(TARGET_DIR, old_filename)  # alte Version ins Release!

# 1. Alte Version ins Release kopieren
shutil.copy2(src_old, release_copy)

# 2. Datei im src-Ordner auf neue Version umbenennen
os.rename(src_old, src_new)

print(f"Release erstellt: {release_copy}")
print(f"src-Version aktualisiert: {src_new}")