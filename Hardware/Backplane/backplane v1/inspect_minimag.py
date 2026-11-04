from pathlib import Path
import sqlite3
import zipfile
import sys
import base64,gzip,json
sys.stdout.reconfigure(errors='replace')

root=Path(r'C:\Users\Ejfso\Documents\GitHub\miniMagSwitch')
con=sqlite3.connect((root/'miniMagSwitch.eprj').as_uri()+'?mode=ro',uri=True)
for row in con.execute("select key,value from attributes where device_uuid='c902f38568d24296a71c25b1640b42c1'"):
    print('HEADER ATTRIBUTE',row)
for title,data in con.execute("select title,dataStr from components where docType=4"):
    raw=gzip.decompress(base64.b64decode(data[6:])).decode() if data.startswith('base64') else data
    if '2.54' in title or 'HDR' in title.upper():
        print('HEADER FOOTPRINT',title)
        for line in raw.splitlines():
            if line.startswith('["PAD",'):
                print(line)
sys.exit()
for title,typ,data in con.execute('select title,docType,dataStr from documents'):
    raw=gzip.decompress(base64.b64decode(data[6:])).decode() if data.startswith('base64') else data
    print('DOCUMENT',title,typ,'LEN',len(raw))
    if typ==3:
        for line in raw.splitlines():
            if any(s in line for s in ('PAD','COMPONENT','ATTR','POLY','RECT')):
                print(line[:1800])
for name,sql in con.execute("select name, sql from sqlite_master where type='table'"):
    if name not in ('components','devices','projects') and ('dataStr' in sql or name in ('pcbs','sheets')):
        print('TABLE',name,sql)
        for row in con.execute('select * from "'+name+'" limit 1'):
            print(str(row)[:1000])
with zipfile.ZipFile(root/'fabrication/miniMagSwitch.zip') as z:
    print('ZIP',z.namelist())
    for n in z.namelist():
        if n.lower().endswith(('.gko','.drl','.txt','.gml')):
            print('\nFILE',n,'\n',z.read(n).decode(errors='replace')[:16000])
