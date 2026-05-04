#pragma once
// ---------------------------------------------------------------------------
// HTTPS / TLS context — HttpCtx struct and ESP-IDF httpd_ssl helpers.
//
// This header must be included BEFORE any function definitions in the .ino
// so that the Arduino prototype generator sees HttpCtx before the prototypes
// for functions that reference it.
//
// Two I/O paths are supported via flags on HttpCtx:
//   isWS  = true   → WebServer (HTTP port 80) — handler uses global `server`
//                    directly for headers/output; HttpCtx is mostly unused.
//   isIDF = true   → ESP-IDF httpd_ssl_server (HTTPS port 443) — populated
//                    by initFromIDFReq(); accessor and emit methods on
//                    HttpCtx wrap the IDF request/response APIs.
// ---------------------------------------------------------------------------

#include <esp_https_server.h>   // ESP-IDF TLS server (httpd_ssl_*, httpd_uri_t)
#include <esp_http_server.h>    // httpd_req_t, httpd_resp_*, httpd_query_*

// ---------------------------------------------------------------------------
// Runtime constants
// ---------------------------------------------------------------------------

#define HTTPS_MAX_HDRS  12  // max response headers tracked
#define HTTPS_MAX_ARGS   8  // max query-string / form KV pairs

// ---------------------------------------------------------------------------
// HttpCtx — request/response context for HTTPS handlers.
//
// Accessors and emit methods (header/arg/send/sendProgmem) dispatch on the
// isIDF flag. They are not called on the WS path — the WS path branches on
// ctx.isWS at the call site and uses server.* directly.
// ---------------------------------------------------------------------------

struct HttpCtx {
  bool         isWS;     // WebServer path (HTTP port 80)
  bool         isIDF;    // ESP-IDF httpd path (HTTPS port 443)

  // Request body — populated by initFromIDFReq() for IDF POSTs.
  String       body;

  // ESP-IDF httpd path
  httpd_req_t* idfReq;   // active IDF request (set by initFromIDFReq)

  struct KV { String k, v; };
  KV  args[HTTPS_MAX_ARGS];   // parsed from query string + form body
  int nArgs;

  struct RespHdr { String k, v; };
  RespHdr respHdrs[HTTPS_MAX_HDRS];
  int     nRespHdrs;
  bool    respStarted;

  HttpCtx() : isWS(false), isIDF(false), idfReq(nullptr),
              nArgs(0), nRespHdrs(0), respStarted(false) {}

  // ----- request accessors (IDF path) -----
  String header(const String& k) const {
    if (!isIDF || !idfReq) return String();
    size_t len = httpd_req_get_hdr_value_len(idfReq, k.c_str());
    if (len == 0) return String();
    char* buf = (char*)malloc(len + 1);
    if (!buf) return String();
    String result;
    if (httpd_req_get_hdr_value_str(idfReq, k.c_str(), buf, len + 1) == ESP_OK)
      result = String(buf);
    free(buf);
    return result;
  }
  bool hasHeader(const String& k) const {
    if (!isIDF || !idfReq) return false;
    return httpd_req_get_hdr_value_len(idfReq, k.c_str()) > 0;
  }
  String arg(const String& k) const {
    for (int i = 0; i < nArgs; i++)
      if (args[i].k.equals(k)) return args[i].v;
    return String();
  }
  bool hasArg(const String& k) const {
    for (int i = 0; i < nArgs; i++)
      if (args[i].k.equals(k)) return true;
    return false;
  }

  // ----- response helpers (IDF path) -----
  void addRespHdr(const String& k, const String& v) {
    if (nRespHdrs < HTTPS_MAX_HDRS) {
      respHdrs[nRespHdrs].k = k;
      respHdrs[nRespHdrs].v = v;
      nRespHdrs++;
    }
  }

  void flushRespHdrs(int code, const char* ct, int /*contentLen*/ = -1) {
    if (respStarted || !isIDF || !idfReq) { respStarted = true; return; }
    respStarted = true;
    const char* reason =
      (code == 200) ? "OK"               :
      (code == 301) ? "Moved Permanently" :
      (code == 302) ? "Found"            :
      (code == 303) ? "See Other"        :
      (code == 400) ? "Bad Request"      :
      (code == 401) ? "Unauthorized"     :
      (code == 404) ? "Not Found"        :
      (code == 413) ? "Payload Too Large" :
      (code == 423) ? "Locked"           :
      (code == 429) ? "Too Many Requests" : "Unknown";
    char status[40];
    snprintf(status, sizeof(status), "%d %s", code, reason);
    httpd_resp_set_status(idfReq, status);
    httpd_resp_set_type(idfReq, ct);
    for (int i = 0; i < nRespHdrs; i++)
      httpd_resp_set_hdr(idfReq, respHdrs[i].k.c_str(), respHdrs[i].v.c_str());
    // Content-Length set automatically by httpd_resp_send / httpd_resp_send_chunk.
  }

  void send(int code, const char* ct, const String& bodyStr) {
    flushRespHdrs(code, ct, (int)bodyStr.length());
    if (isIDF && idfReq)
      httpd_resp_send(idfReq, bodyStr.c_str(), (ssize_t)bodyStr.length());
  }

  void sendProgmem(int code, const char* ct, const char* pgmData) {
    size_t len = strlen_P(pgmData);
    flushRespHdrs(code, ct, (int)len);
    if (!isIDF || !idfReq) return;
    // Stream PROGMEM in 512-byte chunks via chunked transfer encoding.
    const size_t CHUNK = 512;
    char buf[CHUNK];
    size_t sent = 0;
    while (sent < len) {
      size_t n = ((len - sent) < CHUNK) ? (len - sent) : CHUNK;
      memcpy_P(buf, pgmData + sent, n);
      httpd_resp_send_chunk(idfReq, buf, (ssize_t)n);
      sent += n;
    }
    httpd_resp_send_chunk(idfReq, nullptr, 0);  // terminate chunked transfer
  }
};

// ---------------------------------------------------------------------------
// URL-decode a percent-encoded string (replaces %XX and '+' with space).
// ---------------------------------------------------------------------------
inline String urlDecode(const String& in) {
  String out;
  out.reserve(in.length());
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in.charAt(i);
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < in.length()) {
      char h[3] = { in.charAt(i + 1), in.charAt(i + 2), '\0' };
      out += (char)strtol(h, nullptr, 16);
      i += 2;
    } else {
      out += c;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Parse key=value pairs from a query string or URL-encoded form body into ctx.
// ---------------------------------------------------------------------------
inline void parseKVPairs(const String& src, HttpCtx& ctx) {
  int start = 0;
  while (start <= (int)src.length()) {
    int eq  = src.indexOf('=', start);
    if (eq  < 0) break;
    int amp = src.indexOf('&', eq);
    if (amp < 0) amp = (int)src.length();
    if (ctx.nArgs < HTTPS_MAX_ARGS) {
      ctx.args[ctx.nArgs].k = urlDecode(src.substring(start, eq));
      ctx.args[ctx.nArgs].v = urlDecode(src.substring(eq + 1, amp));
      ctx.nArgs++;
    }
    start = amp + 1;
  }
}

// ---------------------------------------------------------------------------
// Populate HttpCtx from an ESP-IDF httpd_req_t.
//
// Reads URL query string and (for form-encoded POSTs) the request body into
// ctx.args[]. Body is also kept in ctx.body for handlers that parse JSON
// directly. Body cap (8 KB) matches the legacy raw-socket path.
// ---------------------------------------------------------------------------
inline void initFromIDFReq(httpd_req_t* req, HttpCtx& ctx) {
  ctx        = HttpCtx();
  ctx.isIDF  = true;
  ctx.idfReq = req;

  // URL query string → args[]
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen > 0) {
    char* qbuf = (char*)malloc(qlen + 1);
    if (qbuf) {
      if (httpd_req_get_url_query_str(req, qbuf, qlen + 1) == ESP_OK) {
        String q(qbuf);
        parseKVPairs(q, ctx);
      }
      free(qbuf);
    }
  }

  // POST body. httpd_req_recv may return short reads, so loop until full.
  int bodyLen = req->content_len;
  if (bodyLen > 0 && bodyLen <= 8192) {
    char* bodyBuf = (char*)malloc(bodyLen + 1);
    if (bodyBuf) {
      int total = 0;
      while (total < bodyLen) {
        int r = httpd_req_recv(req, bodyBuf + total, bodyLen - total);
        if (r <= 0) break;   // 0 = closed; <0 = error/timeout
        total += r;
      }
      if (total > 0) {
        bodyBuf[total] = '\0';
        ctx.body = String(bodyBuf);
        // Form-encoded → args[]
        String ct = ctx.header("Content-Type");
        if (ct.indexOf("application/x-www-form-urlencoded") >= 0)
          parseKVPairs(ctx.body, ctx);
      }
      free(bodyBuf);
    }
  }
}
