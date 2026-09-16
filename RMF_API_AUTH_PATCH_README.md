# RMF API authentication patch

This patch keeps authentication credentials out of source code.

- `HttpTaskForwarder` reads a bearer token from `RMF_API_BEARER_TOKEN`.
- HTTP 4xx responses are treated as definite upstream rejection.
- A rejected job is marked `BLOCKED` and its corridor reservation is released.
- HTTP 5xx and transport failures remain `FORWARD_UNKNOWN` because delivery may
  have succeeded before the response failed.

For the home simulation, generate a short-lived token from the running
`rmf_api_server` container and export it before starting the arbiter. Real robot
deployments must obtain a service token from the site's OpenID Connect provider;
do not use the image's default development secret in production.
