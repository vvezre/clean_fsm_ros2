# 诊断规则：把新鲜度、健康状态和文本信息归一化为诊断结果。

def freshness_level(age_sec, timeout_sec, warning_ratio=0.5):
    # 根据数据年龄返回 OK、WARN 或 ERROR 级别。
    if age_sec < 0.0:
        raise ValueError("age must be non-negative")
    if timeout_sec <= 0.0:
        raise ValueError("timeout must be positive")
    if warning_ratio <= 0.0 or warning_ratio >= 1.0:
        raise ValueError("warning ratio must be between zero and one")
    if age_sec >= timeout_sec:
        return "ERROR"
    if age_sec >= timeout_sec * warning_ratio:
        return "WARN"
    return "OK"


def status_level(ok, freshness, message, stale_is_error=False):
    # 根据健康标志和新鲜度返回诊断级别及规范化摘要。
    if not ok:
        return "ERROR", message
    if freshness == "ERROR":
        return ("ERROR" if stale_is_error else "WARN"), message
    if freshness == "WARN":
        return "WARN", message
    if freshness != "OK":
        raise ValueError("freshness must be OK, WARN, or ERROR")
    return "OK", message
