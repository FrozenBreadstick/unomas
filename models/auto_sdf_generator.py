#!/usr/bin/env python3
import os
from pathlib import Path
from xml.sax.saxutils import escape

if os.path.basename(os.getcwd()) != "models":
    print("This script must be run from the 'models' directory.")
    exit(1)

ROOT = Path.cwd()  # run this from the parent "models" folder

MODEL_CONFIG_TMPL = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Arlo</name>
  </author>
  <description>
    {model_name_clean}
  </description>
</model>
"""

MODEL_SDF_TMPL = """<sdf version="1.7">
  <model name="{model_name}">
    <static>true</static>
    <link name="{model_name}_link">
      <collision name="{model_name}_collision">
        <geometry>
          <mesh>
            <uri>{uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name="{model_name}_visual">
        <geometry>
          <mesh>
            <uri>{uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

def create_files_for_dae(dae_path: Path):
    subfolder = dae_path.parent.name
    model_file = dae_path.name
    model_name = dae_path.stem  # without .dae
    uri = f"model://{subfolder}/{model_file}"

    model_config = dae_path.parent / "model.config"
    model_sdf = dae_path.parent / "model.sdf"

    # Prepare contents
    safe_name = escape(model_name)
    config_text = MODEL_CONFIG_TMPL.format(
        model_name=safe_name,
        model_name_clean=safe_name
    )
    sdf_text = MODEL_SDF_TMPL.format(
        model_name=safe_name,
        uri=escape(uri)
    )

    # Write files if missing, else overwrite if content differs
    for out_path, text in [(model_config, config_text), (model_sdf, sdf_text)]:
        if out_path.exists():
            try:
                existing = out_path.read_text(encoding="utf-8")
            except Exception:
                existing = ""
            if existing == text:
                continue
        out_path.write_text(text, encoding="utf-8")
        print(f"Wrote {out_path}")

def main():
    if ROOT.name != "models":
        print("Warning: current folder is not named 'models'. Running anyway.")
    count = 0
    for dae_path in ROOT.rglob("*.dae"):
        # Skip if .dae sits directly in ROOT with no subfolder
        if dae_path.parent == ROOT:
            print(f"Skipping {dae_path} because it is not in a subfolder")
            continue
        create_files_for_dae(dae_path)
        count += 1
    if count == 0:
        print("No .dae files found under this folder")

if __name__ == "__main__":
    main()
