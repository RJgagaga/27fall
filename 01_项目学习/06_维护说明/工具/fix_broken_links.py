import os,re,urllib.parse
root=os.getcwd()
img=re.compile(r'!\[([^\]]*)\]\(([^)\s]+)\)')
lnk=re.compile(r'(?<!!)\[([^\]]+)\]\(([^)\s]+?)\)')
def exists(dp,l):
    if re.match(r'^(https?:|mailto:|#)',l): return True
    p=urllib.parse.unquote(l.split('#')[0])
    return (not p) or os.path.exists(os.path.normpath(os.path.join(dp,p)))
n=0
for dp,dn,fn in os.walk(root):
    for f in fn:
        if not f.endswith('.md'): continue
        p=os.path.join(dp,f); t=open(p,encoding='utf-8').read()
        parts=re.split(r'(```.*?```)',t,flags=re.S)
        for i in range(0,len(parts),2):
            s=parts[i]
            def ri(m):
                global n
                if exists(dp,m.group(2)): return m.group(0)
                n+=1; return '（图：%s，原截图未随稿上传）'%os.path.basename(urllib.parse.unquote(m.group(2)))
            def rl(m):
                global n
                if exists(dp,m.group(2)): return m.group(0)
                n+=1
                tgt=os.path.basename(m.group(2))
                note='（暂未编写）' if re.search(r'10_澳门科技大学|11_港科大广州',m.group(2)) else ''
                return m.group(1)+note
            s=img.sub(ri,s); s=lnk.sub(rl,s); parts[i]=s
        new=''.join(parts)
        if new!=t: open(p,'w',encoding='utf-8').write(new)
print('fixed',n)
