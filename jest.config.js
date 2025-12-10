module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  setupFilesAfterEnv: ['<rootDir>/docs/src/components/ChatWidget/__tests__/jest.setup.ts'],
  testMatch: [
    '<rootDir>/docs/src/components/ChatWidget/__tests__/**/*.(test|spec).(ts|tsx|js|jsx)'
  ],
  moduleNameMapper: {
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
  },
  transform: {
    '^.+\\.(ts|tsx|js|jsx)$': 'ts-jest',
  },
  collectCoverageFrom: [
    'docs/src/components/ChatWidget/**/*.{ts,tsx}',
    '!docs/src/components/ChatWidget/__tests__/**/*',
    '!docs/src/components/ChatWidget/**/*.d.ts',
  ],
  coverageDirectory: '<rootDir>/coverage',
  coverageReporters: ['text', 'lcov', 'html'],
  coverageThreshold: {
    global: {
      branches: 70,
      functions: 70,
      lines: 70,
      statements: 70,
    },
  },
};