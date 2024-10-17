#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <omp.h>

const int ballCount = 800;
const float minDistanceThreshold = 10.0f;

struct KahanVector2Sum {
    Vector2 sum;
    Vector2 c;
    KahanVector2Sum() : sum{ 0.0f, 0.0f }, c{ 0.0f, 0.0f } {}
    void Add(const Vector2& value) {
        Vector2 y = Vector2Subtract(value, c);
        Vector2 t = Vector2Add(sum, y);
        c = Vector2Subtract(Vector2Subtract(t, sum), y);  // compensate for floating point error
        sum = t;
    }
    Vector2 GetSum() const {
        return sum;
    }
};


struct Ball {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    float radius;
    float mass;
    Color color;
};

const float G = 1000.0f;

Vector2 calculateGravitationalForce(const Ball& source, const Ball& target) {
    Vector2 direction = Vector2Subtract(target.position, source.position);
    float distance = Vector2Length(direction) + minDistanceThreshold;
    direction = Vector2Normalize(direction);
    float forceMagnitude = (G * source.mass * target.mass) / (distance * distance);
    return Vector2Scale(direction, forceMagnitude);
}

bool checkBallCollision(const Ball& ball1, const Ball& ball2) {
    float distance = Vector2Distance(ball1.position, ball2.position);
    return distance < ball1.radius + ball2.radius;
}

void resolveBallCollision(Ball& ball1, Ball& ball2) {
    Vector2 normal = Vector2Normalize(Vector2Subtract(ball2.position, ball1.position));
    Vector2 relativeVelocity = Vector2Subtract(ball2.velocity, ball1.velocity);
    float relativeSpeed = Vector2DotProduct(relativeVelocity, normal);
    if (Vector2Distance(ball1.position, ball2.position) < ball1.radius + ball2.radius) {
        Vector2 direction = Vector2Subtract(ball2.position, ball1.position);
        float distance = Vector2Length(direction);
        float overlap = (ball1.radius + ball2.radius) - distance;
        direction = Vector2Normalize(direction);
        float totalMass = ball1.mass + ball2.mass;
        float ratio1 = ball2.mass / totalMass;
        float ratio2 = ball1.mass / totalMass;
        ball1.position = Vector2Subtract(ball1.position, Vector2Scale(direction, overlap * ratio1));
        ball2.position = Vector2Add(ball2.position, Vector2Scale(direction, overlap * ratio2));
    }

    if (relativeSpeed < 0.0f) {
        float impulse = (2.0f * relativeSpeed) / (ball1.mass + ball2.mass);
        ball1.velocity = Vector2Add(ball1.velocity, Vector2Scale(normal, impulse * ball2.mass));
        ball2.velocity = Vector2Subtract(ball2.velocity, Vector2Scale(normal, impulse * ball1.mass));
    }
}
void handleCollisions(std::vector<Ball>& balls) {
    const int maxIterations = 10;
    for (int iteration = 0; iteration < maxIterations; iteration++) {
        for (int i = 0; i < balls.size(); i++) {
            for (int j = i + 1; j < balls.size(); j++) {
                if (checkBallCollision(balls[i], balls[j])) {
                    resolveBallCollision(balls[i], balls[j]);
                }
            }
        }
    }
}

int main() {
    const int screenWidth = 1600;
    const int screenHeight = 900;
    InitWindow(screenWidth, screenHeight, "physics 2D");
    std::vector<Ball> balls;
    srand(2);
    for (int i = 0; i < ballCount; i++) {
        Ball newBall;
        int random = rand() % 256;
        newBall.position = { (float)(rand() % screenWidth), (float)(rand() % screenHeight) };
        newBall.velocity = { (float)(rand() % 200 - 100), (float)(rand() % 200 - 100) };
        newBall.acceleration = { 0.0f, 0.0f };
        newBall.radius = 5;
        newBall.mass = newBall.radius * ((random / 16) + 1);
        newBall.color = { (unsigned char)(random), (unsigned char)(rand() % 256), (unsigned char)(rand() % 256), 255 };
        balls.push_back(newBall);
    }
    //Ball ball1 = { {screenWidth / 2, screenHeight / 2}, {0.0f, -15.0f}, {0.0f, 0.0f}, 30.0f, 800.0f, BLUE };
    //Ball ball2 = { {(screenWidth / 2) - 200 , (screenHeight / 2) }, {0.0f, 120.0f}, {0.0f, 0.0f}, 10.0f, 100.0f, BLUE };
    //balls.push_back(ball1);
    //balls.push_back(ball2);

    bool isDragging = false;
    int draggedBallIndex = -1;
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            for (int i = 0; i < balls.size(); i++) {
                if (CheckCollisionPointCircle(GetMousePosition(), balls[i].position, balls[i].radius)) {
                    isDragging = true;
                    draggedBallIndex = i;
                    break;
                }
            }
        }
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON) && isDragging) {
            if (isDragging) {
                balls[draggedBallIndex].velocity.x = (GetMouseX() - balls[draggedBallIndex].position.x) / deltaTime;
                balls[draggedBallIndex].velocity.y = ((GetMouseY() - balls[draggedBallIndex].position.y) / deltaTime);
                isDragging = false;
            }
        }
        if (isDragging) {
            balls[draggedBallIndex].position.x = GetMouseX();
            balls[draggedBallIndex].position.y = GetMouseY();
        }
#pragma omp parallel for num_threads(2)
        for (int i = 0; i < balls.size(); i++) {
            Ball& ball = balls[i];
            if (!isDragging) {
                KahanVector2Sum kahanAccelerationSum;
                ball.acceleration = { 0.0f, 0.0f };
                for (const auto& otherBall : balls) {
                    if (&ball != &otherBall) {
                        Vector2 force = calculateGravitationalForce(ball, otherBall);
                        kahanAccelerationSum.Add(Vector2Scale(force, 1.0f / ball.mass));
                    }
                }
                ball.acceleration = kahanAccelerationSum.GetSum();

                KahanVector2Sum kahanVelocitySum;
                kahanVelocitySum.Add(ball.velocity);
                kahanVelocitySum.Add(Vector2Scale(ball.acceleration, deltaTime));
                ball.velocity = kahanVelocitySum.GetSum();

                KahanVector2Sum kahanPositionSum;
                kahanPositionSum.Add(ball.position);
                kahanPositionSum.Add(Vector2Scale(ball.velocity, deltaTime));
                ball.position = kahanPositionSum.GetSum();
            }

            // BOUNDS
            if (ball.position.y > screenHeight + ball.radius) {
                ball.position.y = -ball.radius;
            }
            if (ball.position.y < -ball.radius) {
                ball.position.y = screenHeight + ball.radius;
            }
            if (ball.position.x > screenWidth + ball.radius) {
                ball.position.x = -ball.radius;
            }
            if (ball.position.x < -ball.radius) {
                ball.position.x = screenWidth + ball.radius;
            }
        }
        handleCollisions(balls);
        BeginDrawing();
        ClearBackground(BLACK);
        for (const auto& ball : balls) {
            DrawCircleV(ball.position, ball.radius, ball.color);
        }
        //DrawFPS(0, 0);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
