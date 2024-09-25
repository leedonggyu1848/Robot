using UnityEngine;

public class Bug2Algorithm : MonoBehaviour
{
    
    // input values
    public Transform target;
    
    // constants
    private const float MoveSpeed = 1f; // 이동 속도
    private const float RotationSpeed = 0.5f; // 회전 속도
    private const float SameDistanceThreshold = 1f; // 같은 위치로 판단할 거리
    private const float SameAngleThreshold = 0.1f; // 같은 각도로 판단할 각도
    private const float CollisionThreshold = 2; // 최소 충돌 횟수
    
    // values
    private Vector3 _startPoint;
    private float _hitToGoalDistance;
    private int _collisionCount;

    private MoveState _state = MoveState.Move;

    private enum MoveState
    {
        Move, // motion to goal
        Boundary // boundary following
    };
    
    void Start()
    {
        RotateToTarget();
        _startPoint = Current();
    }

    // Update is called once per frame
    void Update()
    {
        if (IsSamePoint(Target(), Current())) FinishSimulation();
        transform.Translate(MoveSpeed * Time.deltaTime * Vector3.forward);

        switch (_state)
        {
            case MoveState.Move:
                break;
            case MoveState.Boundary:
                if (_collisionCount > CollisionThreshold
                    &&isOnMLine(Current())
                    && Vector3.Distance(Target(), Current()) < _hitToGoalDistance)
                {
                    _state = MoveState.Move;
                    RotateToTarget();
                }
                else
                {
                    transform.Rotate(0, -RotationSpeed, 0);
                }
                break;
        }
    }

    private void OnCollisionEnter(Collision other)
    {
        if (!other.gameObject.CompareTag("Obstacle")) return;
        var normal = other.contacts[0].normal;
        normal.y = 0;
        transform.LookAt(Current() + normal);
        transform.Rotate(0, -45f, 0);
        if (_state == MoveState.Move)
        {
            _state = MoveState.Boundary;
            _collisionCount = 0;
            _hitToGoalDistance = Vector3.Distance(Target(), Current());
        }
    }

    private void OnCollisionStay(Collision other)
    {
        if (!other.gameObject.CompareTag("Obstacle")) return;
        transform.Rotate(0, RotationSpeed * 10, 0);
    }

    private void OnCollisionExit(Collision other)
    {
        if (!other.gameObject.CompareTag("Obstacle")) return;
        _collisionCount++;
    }

    private bool isOnMLine(Vector3 a)
    {
        var aToGoal = Target() - a;
        var startToA = a - _startPoint;
        var judge = Mathf.Abs(aToGoal.x / aToGoal.z - startToA.x / startToA.z);
        return judge < SameAngleThreshold;
    }

    private void RotateToTarget()
    {
        transform.LookAt(Target());
    }
    
    private void FinishSimulation()
    {
        gameObject.SetActive(false);
    }
    
    private bool IsSamePoint(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b) < SameDistanceThreshold;
    }
    
    private Vector3 Target()
    {
        return target.transform.position;
    }

    private Vector3 Current()
    {
        return transform.position;
    }
}
