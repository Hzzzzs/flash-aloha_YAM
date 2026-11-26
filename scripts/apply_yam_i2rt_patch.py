"""Reproduce the two SDK shutdown changes used in the lab; no hardware access."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sdk', type=Path)
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent / 'patches'
    manifest = json.loads((directory / 'i2rt-source.json').read_text())
    sdk = args.sdk.resolve()
    revision = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != manifest['commit']:
        raise SystemExit(f"Expected i2rt {manifest['commit']}, found {revision}")
    states = []
    for name, hashes in manifest['files'].items():
        digest = hashlib.sha256((sdk / name).read_bytes()).hexdigest()
        states.append('patched' if digest == hashes['patched_sha256'] else
                      'upstream' if digest == hashes['upstream_sha256'] else 'unknown')
    if all(state == 'patched' for state in states):
        print('i2rt shutdown patch already applied.')
        return
    if not all(state == 'upstream' for state in states):
        raise SystemExit('SDK files have other or partial changes; left untouched.')
    command = ['git', '-C', str(sdk), 'apply']
    patch = str(directory / manifest['patch'])
    subprocess.run(command + ['--check', patch], check=True)
    subprocess.run(command + [patch], check=True)
    print('Applied the validated i2rt shutdown patch.')


if __name__ == '__main__':
    main()
