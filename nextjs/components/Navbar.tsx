"use client"

import Link from "next/link"
import { ThemeToggle } from "./ThemeToggle"
import { Button } from "./ui/button"
import { BookOpen } from "lucide-react"
import { useSession, signOut } from "@/lib/auth-client"

export function Navbar() {
    const { data: session } = useSession()

    const handleLogout = async () => {
        await signOut()
        window.location.href = "/"
    }

    return (
        <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-white/10">
            <div className="mx-auto max-w-7xl px-4 sm:px-6 lg:px-8">
                <div className="flex h-16 items-center justify-between">
                    <Link href="/" className="flex items-center gap-2 font-heading font-bold text-lg hover:opacity-80 transition-opacity">
                        <div className="p-2 rounded-lg bg-gradient-primary">
                            <BookOpen className="h-5 w-5 text-white" />
                        </div>
                        <span className="text-gradient-primary">Physical AI</span>
                    </Link>

                    <div className="hidden md:flex items-center gap-6">
                        <Link href="/book/intro" className="text-sm font-medium hover:text-accent transition-colors">
                            Start Learning
                        </Link>
                        {session ? (
                            <Button variant="outline" size="sm" onClick={handleLogout}>
                                Logout
                            </Button>
                        ) : (
                            <>
                                <Link href="/login">
                                    <Button variant="outline" size="sm">Login</Button>
                                </Link>
                                <Link href="/signup">
                                    <Button variant="primary" size="sm">Sign Up</Button>
                                </Link>
                            </>
                        )}
                        <ThemeToggle />
                    </div>
                </div>
            </div>
        </nav>
    )
}
