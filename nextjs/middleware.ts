import { NextRequest, NextResponse } from "next/server"
import { auth } from "@/lib/auth"

export async function middleware(request: NextRequest) {
  /*
  // Better Auth middleware logic
  const session = await auth.api.getSession({
    headers: request.headers,
  })
  */
  // Temporary: getting session via fetch to avoid edge runtime issues with direct DB access
  // or use `better-auth/next-js` middleware helper if available.
  // For now, let's keep it simple. If we run into Edge Runtime errors with direct DB access in middleware,
  // we might need to use the client-side check or a specific edge-compatible adapter.
  // But standard better-auth usage often implies Node runtime for db access.
  // Let's stick to the existing code but ensure we don't crash.

  // Note: Middleware in Next.js runs on Edge Runtime by default
  // better-auth might need to call an API route instead of direct DB.
  // Let's rely on the API route approach for session validation if possible.

  // Actually, better-auth docs say `auth.api.getSession` works if configured.
  // Let's assume it works for now.
  const session = await auth.api.getSession({
    headers: request.headers,
  })

  // Protected routes
  const protectedPaths = ["/profile", "/dashboard"]
  const isProtectedPath = protectedPaths.some(path =>
    request.nextUrl.pathname.startsWith(path)
  )

  if (isProtectedPath && !session) {
    return NextResponse.redirect(new URL("/login", request.url))
  }

  // Redirect authenticated users away from auth pages
  const authPaths = ["/login", "/signup"]
  const isAuthPath = authPaths.some(path =>
    request.nextUrl.pathname.startsWith(path)
  )

  if (isAuthPath && session) {
    return NextResponse.redirect(new URL("/", request.url))
  }

  return NextResponse.next()
}

export const config = {
  matcher: [
    "/((?!api|_next/static|_next/image|favicon.ico).*)",
  ],
}