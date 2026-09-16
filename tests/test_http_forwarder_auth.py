from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
import threading
import unittest

from traffic_control.task_gate import HttpTaskForwarder


class _RecordingHandler(BaseHTTPRequestHandler):
    authorization: str | None = None

    def do_POST(self) -> None:
        type(self).authorization = self.headers.get("Authorization")
        length = int(self.headers.get("Content-Length", "0"))
        self.rfile.read(length)
        body = b'{"success": true}'
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:
        return


class HttpTaskForwarderAuthenticationTests(unittest.TestCase):
    def test_bearer_token_environment_variable_is_forwarded(self) -> None:
        _RecordingHandler.authorization = None
        server = ThreadingHTTPServer(("127.0.0.1", 0), _RecordingHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        previous = os.environ.get("RMF_API_BEARER_TOKEN")
        os.environ["RMF_API_BEARER_TOKEN"] = "local-test-token"
        try:
            host, port = server.server_address
            forwarder = HttpTaskForwarder(f"http://{host}:{port}/tasks/robot_task")
            result = forwarder({"hello": "world"})
        finally:
            if previous is None:
                os.environ.pop("RMF_API_BEARER_TOKEN", None)
            else:
                os.environ["RMF_API_BEARER_TOKEN"] = previous
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.assertTrue(result["success"])
        self.assertEqual(
            _RecordingHandler.authorization,
            "Bearer local-test-token",
        )


if __name__ == "__main__":
    unittest.main()
