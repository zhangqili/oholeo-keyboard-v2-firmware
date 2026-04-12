keyboard.watch(55,63,64,65);
keyboard.watch(60);
keyboard.watch(66,67,68);
keyboard.watch(13,14,15);
keyboard.watch(30);
keyboard.watch(28, 29);

var directionPreset = [{ x: -1, y: 0 },{ x: 0, y: 1 },{ x: 1, y: 0 },{ x: 0, y: -1 }]
var counter = 0;
var direction = { x: 0, y: 0 };
var nextDirection = { x: 0, y: 0 };
var width = 11;
var height = 4;
var isRunning = false;
var failed = false;
var snake = [{ x: Math.floor(Math.random() * width), y: Math.floor(Math.random() * height) }];
var food = { x: Math.floor(Math.random() * width), y: Math.floor(Math.random() * height) }
var interval = 4000;
var nextTick = 0;
var grid = [[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
            [17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27],
            [31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41],
            [44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54]];

function snakeInit()
{
    keyboard.command(0x24);
    for (var i = 0; i < 69; i++) {
        led.setMode(i, 0)
    }
    isRunning = true;
    nextDirection = directionPreset[Math.floor(Math.random() * 4)];
    snake = [{ x: Math.floor(Math.random() * width), y: Math.floor(Math.random() * height) }];
    spawnFood();
    renderDirection();
}

function snakeExit()
{
    isRunning = false;
    keyboard.command(0x64);
    keyboard.command(0x08);
}

function speedAdd(x)
{
    interval = Math.max(1000, Math.min(8000, interval + x));

}

function hit(point)
{
    for (var i = 0; i < snake.length; i++) {
        if (snake[i].x == point.x && snake[i].y == point.y) {
            return true;
        }
    }
    return false;
}

function spawnFood()
{
    while (true)
    {
        var newX = Math.floor(Math.random() * width);
        var newY = Math.floor(Math.random() * height);
        if (!hit({ x: newX, y: newY }))
        {
            food.x = newX;
            food.y = newY;
            break;
        }
    }
}

function pause() {
    if (nextTick != 0xFFFFFFFF)
    {
        nextTick = 0xFFFFFFFF;
    }
    else
    {
        nextTick = keyboard.getTick() + interval;
    }
}

function render() {
    for (var i = 0; i < grid.length; i++) {
        for (var j = 0; j < grid[0].length; j++) {
            led.setRGB(grid[i][j], 0, 0,0)
        }
    }
    for (var i = 0; i < snake.length; i++)
    {
        if (failed)
        {
            led.setRGB(grid[snake[i].y][snake[i].x], 0x80, 0, 0);
        }
        else
        {
            led.setRGB(grid[snake[i].y][snake[i].x], 0xff, 0xff, 0xff);
        }
    }
    led.setRGB(grid[food.y][food.x], 0xff, 0, 0);
}

function renderDirection() {
    led.setRGB(55, 0, 0, 0);
    led.setMode(55, 1);
    led.setRGB(63, 0, 0, 0);
    led.setMode(63, 1)
    led.setRGB(64, 0, 0, 0);
    led.setMode(64, 1)
    led.setRGB(65, 0, 0, 0);
    led.setMode(65, 1)
    if (nextDirection.x == 0) {
        if (nextDirection.y == -1) {
            led.setRGB(55, 0xff, 0xff, 0xff);
            led.setMode(55, 1)
        }
        if (nextDirection.y == 1) {
            led.setRGB(64, 0xff, 0xff, 0xff);
            led.setMode(64, 1)
        }
    }
    if (nextDirection.y == 0) {
        if (nextDirection.x == 1) {
            led.setRGB(65, 0xff, 0xff, 0xff);
            led.setMode(65, 1)
        }
        if (nextDirection.x == -1) {
            led.setRGB(63, 0xff, 0xff, 0xff);
            led.setMode(63, 1)
        }
    }
}

function update() {
    if (failed)
    {
        return;
    }
    direction = nextDirection;
    var head = snake[snake.length - 1];
    var newHead = { x: (head.x + direction.x + width) % width, y: (head.y + direction.y + height) % height };
    if (hit(newHead))
    {
        failed = true;
        return;
    }
    if (newHead.x == food.x && newHead.y == food.y)
    {
        snake.push(newHead);
        spawnFood();
    }
    else {
        for (var i = 0; i < snake.length-1; i++) {
            snake[i] = snake[i + 1];
        }
        snake[snake.length - 1] = newHead;
    }
}
// Runs per tick.
function loop() {
    if (isRunning == false)
    {
        return;
    }
    var tick = keyboard.getTick();
    if (tick > nextTick) {
        nextTick = tick + interval;
        console.log(nextDirection);
        console.log("nextTick", nextTick);
        update();
        render();
    }
}

function onKeyDown(key) {
    if (!isRunning)
    {
        if (key.id == 30 && keyboard.getLayerIndex() == 2)
        {
            snakeInit();
        }
        return;
    }
    switch (key.id) {
        case 55: // Up
            if (direction.y !== 1) {
                nextDirection = { x: 0, y: -1 };
            }
            renderDirection();
            break;
        case 63: // Left
            if (direction.x !== 1) {
                nextDirection = { x: -1, y: 0 };
            }
            renderDirection();
            break;
        case 64: // Down
            if (direction.y !== -1) {
                nextDirection = { x: 0, y: 1 };
            }
            renderDirection();
            break;
        case 65: // Right
            if (direction.x !== -1) {
                nextDirection = { x: 1, y: 0 };
            }
            renderDirection();
            break;
        case 60: // Space
        case 66:
        case 67:
        case 68:
            pause();
            break;
        case 13:
        case 14:
        case 15:
            snakeExit();
            break;
        case 28:
            speedAdd(1000);
            break;
        case 29:
            speedAdd(-1000);
            break;
    }
}

function onKeyUp(key) {

}
