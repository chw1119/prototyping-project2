const fs = require('fs');
const path = require('path');
const express = require('express');
const bodyParser = require('body-parser'); // POST 데이터 파싱을 위한 모듈 추가
const app = express();
const { exec } = require('child_process');

const foodList = [];


// 'public' 폴더를 static 파일 디렉토리로 설정
app.use(express.static(path.join(__dirname, 'public')));

// EJS를 사용하고 'front' 폴더를 views 디렉토리로 설정
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'front'));

// POST 데이터를 파싱하기 위한 미들웨어 추가
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));

// ROS 2 토픽에 메시지를 발행하는 함수
function publishToROS(topic, message) {
    exec(`ros2 topic pub ${topic} std_msgs/String '{data: "${message}"}'`);
}

// 클라이언트로부터 주문 데이터를 받는 API 엔드포인트
app.post('/order', (req, res) => {
    const { items, total } = req.body;  // 클라이언트로부터 받은 주문 데이터
    const orderId = Date.now(); // 임시로 주문 ID 생성

    console.log(`주문 받음: 주문 ID ${orderId}, 항목: ${items.join(', ')}, 총액: ${total}원`);
    
    foodList.push({
        orderId : orderId,
        content : items.join(', '),
        cost : total
    })
    
    // ROS 2 토픽에 주문 데이터를 발행
    publishToROS('/order_topic', `주문 ID ${orderId}, 항목: ${items.join(', ')}, 총액: ${total}원`);

    res.send('주문이 정상적으로 처리되었습니다.');
});

app.post('/order-get', (req, res)=>{
    
    res.json(foodList);

    foodList = [];
})

// '/client' 경로로 요청이 들어오면 HTML 파일을 응답으로 반환
app.get('/client', function(req, res) {
    res.render('client'); // 'client.ejs' 파일을 렌더링
});

// '/server' 경로로 요청이 들어오면 HTML 파일을 응답으로 반환
app.get('/server', function(req, res) {
    res.render('server'); // 'server.ejs' 파일을 렌더링
});

// 8080 포트에서 서버 실행
app.listen(8080, () => {
    console.log("서버가 8080 포트에서 실행 중입니다.");
});

// 테스트용으로 주문 자동 추가 (실제 상황에서는 서버에서 주문 데이터를 받을 때 호출)
setTimeout(() => {
    addOrder({ id: 1, items: ['라면', '콜라'], total: 4500 });
}, 2000); // 2초 후 첫 주문

setTimeout(() => {
    addOrder({ id: 2, items: ['치킨'], total: 10000 });
}, 5000); // 5초 후 두 번째 주문

// 주문 추가 함수 (임시로 구현)
function addOrder(order) {
    console.log(`테스트용 주문 추가: 주문 ID ${order.id}, 항목: ${order.items.join(', ')}, 총액: ${order.total}원`);
}
