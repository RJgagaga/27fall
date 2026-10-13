#!/usr/bin/env python3
"""生成文件清单（标题、汉字数、英文词数、占位数）。用法：python3 -I 01_项目学习/06_维护说明/工具/gen_inventory.py > 清单.md"""
import os, re
ROOT = os.path.join(os.getcwd(), '01_项目学习') if os.path.isdir('01_项目学习') else os.getcwd()
rows = []
for dp, dn, fn in os.walk(ROOT):
    if '05_学习笔记归档' in dp or '06_维护说明' in dp:
        continue
    for f in sorted(fn):
        if not f.endswith('.md'):
            continue
        p = os.path.join(dp, f)
        t = open(p, encoding='utf-8').read()
        nc = re.sub(r'```.*?```', '', t, flags=re.S)
        m = re.search(r'^# (.+)$', t, flags=re.M)
        rows.append((os.path.relpath(p, ROOT), (m.group(1).strip() if m else ''),
                     len(re.findall(r'[一-鿿]', nc)), len(re.findall(r'[A-Za-z]{2,}', nc)),
                     len(re.findall(r'【待你补全|【待补|【补全|\[FILL', t))))
rows.sort()
print('| 文件 | 标题 | 汉字 | 英文词 | 待补占位 |\n|---|---|---:|---:|---:|')
for r in rows:
    print(f'| `{r[0]}` | {r[1]} | {r[2]} | {r[3]} | {r[4]} |')
print(f'\n合计 {len(rows)} 个文件，汉字 {sum(r[2] for r in rows)}，英文词 {sum(r[3] for r in rows)}，待补占位 {sum(r[4] for r in rows)}')
