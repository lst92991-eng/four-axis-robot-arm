from pathlib import Path
p=Path('simulation/x42s.cpp')
text=p.read_text(encoding='utf-8')
text=text.replace('void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }\n','')
while 'check(' in text:
    start=text.index('check('); i=start+6; depth=1; quote=False; comma=None
    while depth:
        c=text[i]
        if c=='"' and text[i-1]!='\\': quote=not quote
        if not quote:
            if c=='(': depth+=1
            if c==')': depth-=1
            if c==',' and depth==1: comma=i
        i+=1
    condition=text[start+6:comma].strip(); message=text[comma+1:i-1].strip()
    text=text[:start]+'if (!('+condition+')) throw std::runtime_error('+message+')'+text[i:]
p.write_text(text,encoding='utf-8')
