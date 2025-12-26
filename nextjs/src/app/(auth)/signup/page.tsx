"use client";

import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
    Card,
    CardContent,
    CardDescription,
    CardFooter,
    CardHeader,
    CardTitle,
} from "@/components/ui/card";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
} from "@/components/ui/select";
import { authClient } from "@/lib/auth-client";
import Link from "next/link";
import { useRouter } from "next/navigation";
import { Bot } from "lucide-react";

export default function SignupPage() {
    const router = useRouter();
    const [formData, setFormData] = useState({
        name: "",
        email: "",
        password: "",
        expertiseLevel: "beginner",
        hardwareAccess: "simulation-only",
        learningGoal: "",
    });
    const [loading, setLoading] = useState(false);

    const handleSignup = async () => {
        if (!formData.name || !formData.email || !formData.password) {
            alert("Please fill in name, email, and password");
            return;
        }
        setLoading(true);
        await authClient.signUp.email(
            {
                email: formData.email,
                password: formData.password,
                name: formData.name,
            },
            {
                onSuccess: () => {
                    router.push("/dashboard");
                },
                onError: (ctx) => {
                    alert(ctx.error.message);
                    setLoading(false);
                },
            }
        );
    };

    return (
        <div className="min-h-screen bg-black flex items-center justify-center p-4 font-sans">
            <Card className="w-full max-w-md bg-zinc-900 border-zinc-800 text-white">
                <CardHeader className="space-y-1">
                    <div className="flex items-center gap-2 justify-center mb-4">
                        <Bot className="text-cyan-400 w-8 h-8" />
                        <span className="text-xl font-bold">Panaverse ID</span>
                    </div>
                    <CardTitle className="text-2xl text-center">Create an account</CardTitle>
                    <CardDescription className="text-center text-zinc-400">
                        Join the Physical AI revolution.
                    </CardDescription>
                </CardHeader>
                <CardContent className="space-y-4">
                    <div className="space-y-2">
                        <Label htmlFor="name">Full Name</Label>
                        <Input
                            id="name"
                            placeholder="John Doe"
                            className="bg-black border-zinc-700 focus:border-cyan-500"
                            value={formData.name}
                            onChange={(e) => setFormData({ ...formData, name: e.target.value })}
                        />
                    </div>
                    <div className="space-y-2">
                        <Label htmlFor="email">Email</Label>
                        <Input
                            id="email"
                            type="email"
                            placeholder="m@example.com"
                            className="bg-black border-zinc-700 focus:border-cyan-500"
                            value={formData.email}
                            onChange={(e) => setFormData({ ...formData, email: e.target.value })}
                        />
                    </div>
                    <div className="space-y-2">
                        <Label htmlFor="password">Password</Label>
                        <Input
                            id="password"
                            type="password"
                            className="bg-black border-zinc-700 focus:border-cyan-500"
                            value={formData.password}
                            onChange={(e) => setFormData({ ...formData, password: e.target.value })}
                        />
                    </div>

                    <div className="grid grid-cols-2 gap-4">
                        <div className="space-y-2">
                            <Label>Expertise</Label>
                            <Select
                                value={formData.expertiseLevel}
                                onValueChange={(val) => setFormData({ ...formData, expertiseLevel: val })}
                            >
                                <SelectTrigger className="bg-black border-zinc-700">
                                    <SelectValue />
                                </SelectTrigger>
                                <SelectContent className="bg-zinc-900 border-zinc-800 text-white">
                                    <SelectItem value="beginner">Beginner</SelectItem>
                                    <SelectItem value="intermediate">Intermediate</SelectItem>
                                    <SelectItem value="expert">Expert</SelectItem>
                                </SelectContent>
                            </Select>
                        </div>
                        <div className="space-y-2">
                            <Label>Hardware</Label>
                            <Select
                                value={formData.hardwareAccess}
                                onValueChange={(val) => setFormData({ ...formData, hardwareAccess: val })}
                            >
                                <SelectTrigger className="bg-black border-zinc-700">
                                    <SelectValue />
                                </SelectTrigger>
                                <SelectContent className="bg-zinc-900 border-zinc-800 text-white">
                                    <SelectItem value="simulation-only">Sim Only</SelectItem>
                                    <SelectItem value="edge-kit">Jetson Kit</SelectItem>
                                    <SelectItem value="robot">Full Robot</SelectItem>
                                </SelectContent>
                            </Select>
                        </div>
                    </div>

                </CardContent>
                <CardFooter className="flex flex-col gap-4">
                    <Button
                        className="w-full bg-cyan-600 hover:bg-cyan-500 text-white"
                        onClick={handleSignup}
                        disabled={loading}
                    >
                        {loading ? "Creating..." : "Sign Up"}
                    </Button>
                    <div className="text-center text-sm text-zinc-400">
                        Already have an account?{" "}
                        <Link href="/signin" className="text-cyan-400 hover:underline">
                            Sign in
                        </Link>
                    </div>
                </CardFooter>
            </Card>
        </div>
    );
}
