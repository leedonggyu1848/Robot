using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class Bug1Algorithm : MonoBehaviour
{
    // input values
    public Transform target;

    // constants
    private const float MoveSpeed = 1f; // 이동 속도
    private const float RotationSpeed = 0.5f; // 회전 속도
    private const int CollisionThreshold = 2; // 최소 충돌 횟수
    private const float SameDistanceThreshold = 1f; // 같은 위치로 판단할 거리

    // values
    private MoveState _state = MoveState.Move; // 현재 행동 상태
    
    // scan values
    private Vector3 _leavePoint;
    private Vector3 _hitPoint;
    private int _collisionCount; // 스캔 상태 진입 후 frame 숫자
    private float _shortestDistance = float.MaxValue;

    private enum MoveState
    {
        Move, // move to target
        Scan, // scan obstacle
        Rotate // move to leave point
    }

    private void Start()
    {
        RotateToTarget();
    }

    private void Update()
    {
        if (IsSamePoint(Target(), Current())) FinishSimulation();
        transform.Translate(MoveSpeed * Time.deltaTime * Vector3.forward);
        
        switch (_state)
        {
            case MoveState.Move:
                break;
            case MoveState.Scan:
                if (_collisionCount > CollisionThreshold
                    && IsSamePoint(_hitPoint, Current()))
                    _state = MoveState.Rotate;
                else
                    transform.Rotate(0, -RotationSpeed, 0);
                StepOfScan();
                break;
            case MoveState.Rotate:
                if (IsSamePoint(_leavePoint, Current()))
                {
                    _state = MoveState.Move;
                    RotateToTarget();
                }
                else
                    transform.Rotate(0, -RotationSpeed, 0);
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
            _leavePoint = Current();
            _hitPoint = Current();
            _collisionCount = 0;
            _shortestDistance = float.MaxValue;
            _state = MoveState.Scan;
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

    private void StepOfScan()
    {
        if (Vector3.Distance(Current(), Target()) < _shortestDistance)
        {
            _shortestDistance = Vector3.Distance(Current(), Target());
            _leavePoint = Current();
        }
    }

    private bool IsSamePoint(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b) < SameDistanceThreshold;
    }

    private void FinishSimulation()
    {
        gameObject.SetActive(false);
    }

    private void RotateToTarget()
    {
        transform.LookAt(Target());
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
