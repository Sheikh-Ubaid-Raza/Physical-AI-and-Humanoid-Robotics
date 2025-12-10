// Logging utility for the chat widget
export class Logger {
  static log(level: 'info' | 'warn' | 'error' | 'debug', message: string, data?: any) {
    const timestamp = new Date().toISOString();
    const logEntry = {
      timestamp,
      level,
      message,
      data,
      // Add any additional context here
      context: 'ChatWidget'
    };

    // In development, log to console
    if (process.env.NODE_ENV !== 'production') {
      console.log(`[${timestamp}] [${level.toUpperCase()}] ${message}`, data || '');
    }

    // In the future, we could send logs to a centralized logging service
    // For now, we'll just log to console in dev mode
  }

  static info(message: string, data?: any) {
    this.log('info', message, data);
  }

  static warn(message: string, data?: any) {
    this.log('warn', message, data);
  }

  static error(message: string, error?: any, data?: any) {
    this.log('error', message, { error: error?.message || error, stack: error?.stack, ...data });
  }

  static debug(message: string, data?: any) {
    this.log('debug', message, data);
  }
}