import xml.etree.ElementTree as ET
import sys, os

def extract_models(world_file, out_dir="models_extracted"):
    tree = ET.parse(world_file)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        print("❌ No <world> found")
        return

    models = world.findall("model")
    if not models:
        print("⚠️ No <model> found")
        return

    os.makedirs(out_dir, exist_ok=True)

    for model in models:
        name = model.attrib.get("name", "unnamed")
        new_root = ET.Element("sdf", version="1.7")
        new_root.append(model)
        out_file = os.path.join(out_dir, f"{name}.sdf")
        ET.ElementTree(new_root).write(out_file, encoding="utf-8", xml_declaration=True)
        print(f"✅ Wrote {out_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_models.py world1.world [out_dir]")
    else:
        extract_models(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "models_extracted")
