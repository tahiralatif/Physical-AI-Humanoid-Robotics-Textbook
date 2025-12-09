"use client"

import { useState } from "react"
import Link from "next/link"
import { useRouter } from "next/navigation"
import { signUp } from "@/lib/auth-client"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { AlertCircle, Loader2 } from "lucide-react"

export default function SignupPage() {
  const [formData, setFormData] = useState({
    name: "",
    email: "",
    password: "",
    pythonLevel: "",
    rosExperience: "",
    hardwareAccess: "",
    learningGoal: ""
  })
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const router = useRouter()

  const handleChange = (field: string, value: string) => {
    setFormData(prev => ({ ...prev, [field]: value }))
  }

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault()
    setLoading(true)
    setError(null)

    // Basic validation
    if (!formData.pythonLevel || !formData.rosExperience || !formData.hardwareAccess) {
      setError("Please complete all background assessment questions")
      setLoading(false)
      return
    }

    try {
      await signUp.email({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        callbackURL: "/dashboard",
        fetchOptions: {
          onResponse: () => setLoading(false),
          onRequest: () => setLoading(true),
          onError: (ctx) => {
            setError(ctx.error.message)
            setLoading(false)
          },
          onSuccess: async () => {
            // Here we would ideally also save the profile data to our user_profiles table 
            // This will be handled in a follow-up API call or hook
            router.push("/dashboard")
          }
        }
      })
    } catch (err) {
      setError("An unexpected error occurred during signup")
      setLoading(false)
    }
  }

  return (
    <div className="flex items-center justify-center min-h-screen bg-muted/40 px-4 py-8">
      <Card className="w-full max-w-2xl border-t-4 border-purple-500 shadow-xl bg-white/90 backdrop-blur-sm">
        <CardHeader className="space-y-1">
          <CardTitle className="text-2xl font-bold text-center">Create Your Account</CardTitle>
          <CardDescription className="text-center">
            Join the Physical AI course. We'll personalize the content based on your background.
          </CardDescription>
        </CardHeader>
        <CardContent>
          <form onSubmit={handleSignup} className="space-y-6">
            {error && (
              <div className="flex items-center gap-2 p-3 text-sm text-red-600 bg-red-50 rounded-md border border-red-100">
                <AlertCircle className="h-4 w-4" />
                <span>{error}</span>
              </div>
            )}

            <div className="grid md:grid-cols-2 gap-4">
              <div className="space-y-2">
                <Label htmlFor="name">Full Name</Label>
                <Input
                  id="name"
                  placeholder="John Doe"
                  value={formData.name}
                  onChange={(e) => handleChange("name", e.target.value)}
                  required
                />
              </div>
              <div className="space-y-2">
                <Label htmlFor="email">Email</Label>
                <Input
                  id="email"
                  type="email"
                  placeholder="name@example.com"
                  value={formData.email}
                  onChange={(e) => handleChange("email", e.target.value)}
                  required
                />
              </div>
              <div className="space-y-2 md:col-span-2">
                <Label htmlFor="password">Password</Label>
                <Input
                  id="password"
                  type="password"
                  placeholder="********"
                  value={formData.password}
                  onChange={(e) => handleChange("password", e.target.value)}
                  required
                />
                <p className="text-xs text-gray-500">Min 8 chars, uppercase, lowercase, number</p>
              </div>
            </div>

            <div className="space-y-4 border-t pt-4">
              <h3 className="font-semibold text-gray-900">Background Assessment</h3>

              <div className="grid md:grid-cols-2 gap-4">
                <div className="space-y-2">
                  <Label>Python Proficiency</Label>
                  <Select onValueChange={(val) => handleChange("pythonLevel", val)}>
                    <SelectTrigger>
                      <SelectValue placeholder="Select level" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="beginner">Beginner (Basic syntax)</SelectItem>
                      <SelectItem value="intermediate">Intermediate (OOP, Libraries)</SelectItem>
                      <SelectItem value="expert">Expert (Async, Optimization)</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div className="space-y-2">
                  <Label>ROS / Robotics Experience</Label>
                  <Select onValueChange={(val) => handleChange("rosExperience", val)}>
                    <SelectTrigger>
                      <SelectValue placeholder="Select experience" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="none">None / New to Robotics</SelectItem>
                      <SelectItem value="basic">Basic (ROS 1/2 concepts)</SelectItem>
                      <SelectItem value="advanced">Advanced (Built robots)</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div className="space-y-2">
                  <Label>Hardware Access</Label>
                  <Select onValueChange={(val) => handleChange("hardwareAccess", val)}>
                    <SelectTrigger>
                      <SelectValue placeholder="Select hardware" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="none">None (Cloud Simulation only)</SelectItem>
                      <SelectItem value="jetson">NVIDIA Jetson (Nano/Orin)</SelectItem>
                      <SelectItem value="rtx">High-end GPU Workstation (RTX)</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <div className="space-y-2">
                  <Label>Primary Learning Goal</Label>
                  <Select onValueChange={(val) => handleChange("learningGoal", val)}>
                    <SelectTrigger>
                      <SelectValue placeholder="Select goal" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="job">Career / Job Switch</SelectItem>
                      <SelectItem value="startup">Building a Startup</SelectItem>
                      <SelectItem value="research">Academic Research</SelectItem>
                      <SelectItem value="hobby">Hobby / Personal Interest</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>
            </div>

            <Button
              type="submit"
              className="w-full bg-gradient-to-r from-indigo-600 to-purple-600 hover:from-indigo-700 hover:to-purple-700 text-white mt-6 transition-all duration-300"
              disabled={loading}
            >
              {loading ? (
                <>
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  Creating Account...
                </>
              ) : (
                "Complete Signup & Personalize"
              )}
            </Button>
          </form>
        </CardContent>
        <CardFooter className="flex justify-center">
          <div className="text-sm text-gray-500">
            Already have an account?{" "}
            <Link href="/login" className="text-indigo-600 hover:underline font-medium">
              Sign In
            </Link>
          </div>
        </CardFooter>
      </Card>
    </div>
  )
}
