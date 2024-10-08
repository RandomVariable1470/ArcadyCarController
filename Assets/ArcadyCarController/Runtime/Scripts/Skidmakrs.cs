using UnityEngine;

namespace Arcady
{
    public class Skidmakrs : MonoBehaviour
    {
        [SerializeField] private TrailRenderer skidmark;

        private Rigidbody _rb;
        private ArcadyController _controller;

        private void Start()
        {
            _rb = GetComponent<Rigidbody>();
            _controller = GetComponentInParent<ArcadyController>();
            
            skidmark.transform.localPosition = new Vector3(0, -transform.GetComponent<SphereCollider>().radius, 0);
        }

        private void Update()
        {
            Vector3 velocity = transform.InverseTransformDirection(_rb.velocity);

            if (_controller.IsGrounded())
            {
                skidmark.emitting = Mathf.Abs(velocity.x) > _controller.DriftSteerThreshold + 0.1f;
            }
            else
            {
                skidmark.emitting = false;
            }
        }
    }
}