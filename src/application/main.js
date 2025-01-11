const fs = require('fs');
const express = require('express');
const app = express();

// 'public' 폴더를 static 파일 디렉토리로 설정
app.use(express.static('./front'));

// '/client' 경로로 요청이 들어오면 HTML 파일을 응답으로 반환
app.get('/client', function(req, res) {
    res.render('index.html');
});

// 8080 포트에서 서버 실행
app.listen(8080, () => {
    console.log("서버가 8080 포트에서 실행 중입니다.");
});
