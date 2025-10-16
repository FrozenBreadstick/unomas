import os
import re

# Regex to match the transform matrix content
# Captures the 16 float values inside <matrix sid="transform"> ... </matrix>
matrix_pattern = re.compile(
    r'(<matrix\s+sid="transform"\s*>)([\s\dEe\+\-\.]+)(</matrix>)'
)

if os.path.basename(os.getcwd()) != "models":
    print("This script must be run from the 'models' directory.")
    exit(1)

def zero_transform_in_text(text):
    def replacer(match):
        prefix, numbers_str, suffix = match.groups()
        numbers = numbers_str.strip().split()

        # Ensure it's a 4x4 matrix
        if len(numbers) == 16:
            numbers[3] = '0'
            numbers[7] = '0'
            numbers[11] = '0'
            new_numbers_str = ' '.join(numbers)
            return f"{prefix}{new_numbers_str}{suffix}"
        else:
            # Don't touch malformed matrices
            return match.group(0)

    return matrix_pattern.sub(replacer, text)

def process_dae_file(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            original = f.read()

        modified = zero_transform_in_text(original)

        if modified != original:
            # Backup original
            with open(path + '.bak', 'w', encoding='utf-8') as backup:
                backup.write(original)

            # Write modified
            with open(path, 'w', encoding='utf-8') as f:
                f.write(modified)

            print(f"[OK] Updated transform in: {path}")
        else:
            print(f"[SKIP] No transform matrix found in: {path}")
    except Exception as e:
        print(f"[ERROR] {path}: {e}")

# Walk directory
for root, dirs, files in os.walk(os.getcwd()):
    for file in files:
        if file.lower().endswith('.dae'):
            full_path = os.path.join(root, file)
            process_dae_file(full_path)

print("Done. All COLLADA transforms zeroed.")
