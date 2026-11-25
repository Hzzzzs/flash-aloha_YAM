"""Apply the bundled encoder-options patch to this interpreter's robologger 0.1.7.

Run after installing requirements-yam-capture.txt, with all capture processes stopped.
No network access, device access or motor operations. Repeated runs are a no-op.
"""
from importlib.metadata import distribution
from pathlib import Path
import subprocess


def main():
    package = distribution('robologger')
    if package.version != '0.1.7':
        raise SystemExit(f'Patch is for robologger 0.1.7; found {package.version}')
    patch = Path(__file__).resolve().parent / 'patches/robologger-0.1.7-encoder-options.patch'
    command = ['patch', '--batch', '-p1', '-d', str(package.locate_file('')),
               '-i', str(patch)]
    check = subprocess.run(command + ['--dry-run', '--forward'], capture_output=True, text=True)
    if check.returncode == 0:
        subprocess.run(command + ['--forward'], check=True)
        print('Applied robologger encoder-options patch.')
        return
    reverse = subprocess.run(command + ['--dry-run', '--reverse'], capture_output=True, text=True)
    if reverse.returncode == 0:
        print('robologger encoder-options patch already applied.')
        return
    raise SystemExit('Installed robologger differs from the patch; no changes made.\n'
                     + check.stdout + check.stderr)


if __name__ == '__main__':
    main()
