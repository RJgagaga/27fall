#!/usr/bin/env python3
"""检查 01_项目学习 下所有 Markdown：断链、代码围栏、$$ 配对、\\[ \\]、手机号邮箱、README 自查表标题。
用法（在仓库根目录）：python3 -I 01_项目学习/06_维护说明/工具/check_all.py
退出码 0 = 无问题。"""
import os, re, sys, urllib.parse, collections
ROOT = os.path.join(os.getcwd(), '01_项目学习') if os.path.isdir('01_项目学习') else os.getcwd()
SKIP = '05_学习笔记归档'
link = re.compile(r'(?<!!)\[[^\]]*\]\(([^)\s]+?)(?:\s+"[^"]*")?\)|!\[[^\]]*\]\(([^)\s]+?)\)')
problems = collections.defaultdict(list)
files = []
for dp, dn, fn in os.walk(ROOT):
    for f in fn:
        if f.endswith('.md'):
            files.append(os.path.join(dp, f))
total = 0
for p in files:
    rel = os.path.relpath(p, ROOT)
    t = open(p, encoding='utf-8').read()
    nocode = re.sub(r'```.*?```', '', t, flags=re.S)
    for m in link.finditer(nocode):
        l = m.group(1) or m.group(2)
        if re.match(r'^(https?:|mailto:|#)', l):
            continue
        path = urllib.parse.unquote(l.split('#')[0])
        if not path:
            continue
        total += 1
        if not os.path.exists(os.path.normpath(os.path.join(os.path.dirname(p), path))):
            problems['断链'].append(f'{rel}: {l}')
    if SKIP in rel:
        continue
    if len(re.findall(r'^```', t, flags=re.M)) % 2:
        problems['代码围栏不成对'].append(rel)
    if nocode.count('$$') % 2:
        problems['$$不成对'].append(rel)
    if re.search(r'^\s*\\\[|\\\]\s*$', nocode, flags=re.M):
        problems['使用了\\[ \\]公式定界'].append(rel)
    if re.search(r'(?<!\d)1[3-9]\d[ -]?\d{4}[ -]?\d{4}(?!\d)', nocode) or re.search(r'@(outlook|sztu\.edu|cuhk\.edu|gmail)', nocode):
        problems['疑似手机号/邮箱'].append(rel)
    if os.path.basename(p) == 'README.md' and '01_项目课程' in rel and rel.count(os.sep) == 2:
        if not re.search(r'^## 实机细节自查表', t, flags=re.M):
            problems['项目README缺"## 实机细节自查表"'].append(rel)
print(f'检查文件 {len(files)} 个，相对链接 {total} 条')
bad = 0
for k, v in problems.items():
    print(f'\n[{k}] {len(v)} 处')
    for x in v[:50]:
        print('  ', x)
    bad += len(v)
print('\n结果：', '无问题' if not bad else f'共 {bad} 个问题')
sys.exit(1 if bad else 0)
