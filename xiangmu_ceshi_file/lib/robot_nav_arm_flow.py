#!/usr/bin/env python3
"""
导航客户端 V4.0 (仅 NavigationClient)
"""
import json, time, uuid
from urllib import error, request
from typing import Any, Dict

NAV_COMMAND_CODE = "navigation"
NAV_FINAL_SUCCESS = {"finished"}
NAV_FINAL_FAILED = {"error", "terminated", "unprocess"}

class NavigationClient:
    def __init__(self, base_url: str, timeout_seconds: float = 8.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_seconds = timeout_seconds

    def _post_json(self, url: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
        try:
            with request.urlopen(req, timeout=self.timeout_seconds) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"HTTP 错误: {exc.code}") from exc
        except error.URLError as exc:
            raise RuntimeError(f"网络错误: {exc.reason}") from exc

    def submit_navigation(self, pose: Dict[str, Any], task_id: str) -> None:
        payload = {
            "task_id": task_id,
            "task_command_info": [{
                "command_id": f"{task_id}-cmd-nav",
                "command_code": NAV_COMMAND_CODE,
                "command_param": pose
            }]
        }
        url = f"{self.base_url}/api/s1-agent/v1/task/submit"
        body = self._post_json(url, payload)
        if not body.get("success"):
            raise RuntimeError(f"导航提交失败: {body}")

    def wait_navigation(self, task_id: str, timeout: float, interval: float) -> None:
        deadline = time.time() + timeout
        url = f"{self.base_url}/api/s1-agent/v1/task/query"
        while time.time() < deadline:
            body = self._post_json(url, {"task_id": task_id})
            if not body.get("success"):
                raise RuntimeError(f"查询导航失败: {body}")
            status = (body.get("result") or {}).get("task_status")
            if status in NAV_FINAL_SUCCESS:
                return
            if status in NAV_FINAL_FAILED:
                raise RuntimeError(f"导航失败: {status}")
            time.sleep(interval)
        raise TimeoutError(f"导航超时: {task_id}")