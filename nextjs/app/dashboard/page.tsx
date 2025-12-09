import { auth } from "@/lib/auth"
import { headers } from "next/headers"
import { redirect } from "next/navigation"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"

export default async function DashboardPage() {
    const session = await auth.api.getSession({
        headers: await headers()
    })

    if (!session) {
        redirect("/login")
    }

    return (
        <div className="container mx-auto p-8">
            <h1 className="text-3xl font-bold mb-8">Welcome, {session.user.name}</h1>

            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                <Card>
                    <CardHeader>
                        <CardTitle>My Progress</CardTitle>
                    </CardHeader>
                    <CardContent>
                        <p className="text-muted-foreground">0% Completed</p>
                        <div className="w-full bg-secondary h-2 mt-2 rounded-full overflow-hidden">
                            <div className="bg-primary h-full w-0"></div>
                        </div>
                    </CardContent>
                </Card>

                <Card>
                    <CardHeader>
                        <CardTitle>Recent Activity</CardTitle>
                    </CardHeader>
                    <CardContent>
                        <p className="text-sm text-muted-foreground">No recent activity.</p>
                    </CardContent>
                </Card>

                <Card>
                    <CardHeader>
                        <CardTitle>Next Module</CardTitle>
                    </CardHeader>
                    <CardContent>
                        <p className="font-medium">Module 1: ROS 2 Fundamentals</p>
                        <p className="text-sm text-muted-foreground mt-1">Start your journey into Robot Operating System.</p>
                    </CardContent>
                </Card>
            </div>
        </div>
    )
}
