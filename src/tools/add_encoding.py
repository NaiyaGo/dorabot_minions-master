#!/usr/bin/env python3
# Script to add UTF-8 encoding declaration to Python files missing it.
import os
import io

ROOT = os.path.dirname(os.path.dirname(__file__))  # src/
count = 0
for dirpath, dirnames, filenames in os.walk(ROOT):
    # skip virtualenv-like dirs if any
    for fname in filenames:
        if not fname.endswith('.py'):
            continue
        path = os.path.join(dirpath, fname)
        try:
            with open(path, 'rb') as fh:
                raw = fh.read()
        except Exception as e:
            print('skip', path, 'err', e)
            continue
        # get first two lines decoded as utf-8-ignore
        lines = raw.splitlines()
        first = lines[0].decode('utf-8', 'ignore') if len(lines) > 0 else ''
        second = lines[1].decode('utf-8', 'ignore') if len(lines) > 1 else ''
        if 'coding' in first.lower() or 'coding' in second.lower():
            continue
        # prepare new content: respect shebang
        encoding_line = '# -*- coding: utf-8 -*-\n'
        if first.startswith('#!'):
            new = first + '\n' if not first.endswith('\n') else first + ''
            # insert encoding after shebang
            rest = b'\n'.join(lines[1:])
            new_bytes = (first + '\n' + encoding_line).encode('utf-8') + rest
        else:
            new_bytes = (encoding_line).encode('utf-8') + raw
        # write back
        try:
            with open(path, 'wb') as fh:
                fh.write(new_bytes)
            count += 1
            print('patched', path)
        except Exception as e:
            print('failed', path, e)

print('Done. files modified:', count)
