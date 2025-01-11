const fs = require('fs');
const path = require('path');
const express = require('express');
const app = express();

// 'public' 폴더를 static 파일 디렉토리로 설정
app.use(express.static(path.join(__dirname, 'public')));

// EJS를 사용하고 'front' 폴더를 views 디렉토리로 설정
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'front'));

// '/client' 경로로 요청이 들어오면 HTML 파일을 응답으로 반환
app.get('/client', function(req, res) {
    res.render('client'); // 'client.ejs' 파일을 렌더링
});

app.get('/server', function(req, res) {
    res.render('server'); // 'server.ejs' 파일을 렌더링
});

// 8080 포트에서 서버 실행
app.listen(8080, () => {
    console.log("서버가 8080 포트에서 실행 중입니다.");
});
