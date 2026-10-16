const http = require('http'), fs = require('fs'), path = require('path');
http.createServer((req, res) => {
  const url = req.url.split('?')[0];
  const f = path.join(__dirname, url === '/' ? 'page_test.html' : url.replace(/^\//, ''));
  fs.readFile(f, (e, d) => {
    if (e) { res.writeHead(404); res.end('not found'); return; }
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.end(d);
  });
}).listen(8765, '127.0.0.1', () => console.log('listening on http://127.0.0.1:8765/page_test.html'));
